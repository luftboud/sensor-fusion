// Measures success criteria 2 and 3 of docs/project.md on the synthetic drive
// of sim/synthetic_drive.h, once for every seed:
//
//   criterion 2: the filter RMSE over the whole route is at least 30% below
//                the RMSE of the raw GNSS fixes (drive without an outage);
//   criterion 3: after the 15 s GNSS outage, the error without the correction
//                is at least 3 times the error at reacquisition with it.
//
//   accuracy [--seeds N] [--out DIR]
//
// Writes accuracy.json and one verdict line per criterion to stderr. See
// docs/accuracy.md.

#include <sys/stat.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "sensor_fusion/ekf/ekf.h"
#include "sensor_fusion/ekf/metrics.h"
#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"
#include "sensor_fusion/ekf/targets.h"
#include "sensor_fusion/sim/synthetic_drive.h"

namespace sensor_fusion {
namespace {

constexpr int kExitOk = 0;
constexpr int kExitFailure = 1;
constexpr int kExitUsage = 2;

constexpr std::string_view kProgram = "accuracy";
constexpr std::string_view kMetricsName = "accuracy.json";
constexpr std::size_t kDefaultSeeds = 20;
constexpr std::size_t kMaxSeeds = 1000;

constexpr std::string_view kUsage =
    "usage: accuracy [--seeds N] [--out DIR]\n"
    "\n"
    "  --seeds N   number of seeds, 1 to 1000; the seeds are 1..N "
    "(default 20)\n"
    "  --out DIR   output directory, default build/report/accuracy\n"
    "\n"
    "Writes accuracy.json. Stdout stays empty; stderr gets one verdict line\n"
    "per criterion.\n"
    "Exit status: 0 done, 1 internal or write error, 2 bad arguments.\n";

struct Options {
  std::size_t seeds = kDefaultSeeds;
  std::string out_dir = "build/report/accuracy";
  bool help = false;
};

// Prints "accuracy: message" to stderr.
void Report(std::string_view message) {
  std::cerr << kProgram << ": " << message << '\n';
}

// Parses an integer in [1, kMaxSeeds]. Returns false, after printing why,
// when the value is not one.
bool ParseSeedCount(std::string_view value, std::size_t& seeds) {
  // An owned copy has a null-terminated buffer, so its data() pair is safe.
  const std::string text(value);
  std::size_t parsed = 0;
  const char* const begin = text.data();
  const char* const end = begin + text.size();
  const auto [ptr, ec] = std::from_chars(begin, end, parsed);
  if (ec != std::errc() || ptr != end || parsed == 0 || parsed > kMaxSeeds) {
    Report("--seeds needs a whole number from 1 to " +
           std::to_string(kMaxSeeds) + ", got '" + std::string(value) + "'");
    return false;
  }
  seeds = parsed;
  return true;
}

// Reads "--flag value" pairs. Returns std::nullopt, after printing why, on a
// bad command line.
std::optional<Options> ParseArguments(std::span<char* const> args) {
  Options options;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string_view flag = args[i];
    if (flag == "--help" || flag == "-h") {
      options.help = true;
      return options;
    }
    if (i + 1 == args.size()) {
      Report(std::string(flag) + " needs a value");
      return std::nullopt;
    }
    const std::string_view value = args[++i];
    if (flag == "--seeds") {
      if (!ParseSeedCount(value, options.seeds)) {
        return std::nullopt;
      }
    } else if (flag == "--out") {
      if (value.empty()) {
        Report("--out needs a directory name");
        return std::nullopt;
      }
      options.out_dir = std::string(value);
    } else {
      Report("unknown flag '" + std::string(flag) + "'");
      return std::nullopt;
    }
  }
  return options;
}

// Output helpers, the same as in kitti_run.cpp.
// The shortest text that reads back as the same double, as std::to_chars
// gives it. The same value always gives the same text.
void AppendNumber(double value, std::string& line) {
  std::array<char, 32> buffer = {};
  const std::to_chars_result result =
      std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
  // 32 characters hold every double; ec is set only if they did not.
  if (result.ec == std::errc()) {
    line.append(buffer.data(), result.ptr);
  }
}

// Appends a JSON number, or null for a missing or non-finite value.
void AppendJsonNumber(std::optional<double> value, std::string& text) {
  if (value.has_value() && std::isfinite(*value)) {
    AppendNumber(*value, text);
  } else {
    text += "null";
  }
}

// Creates path and its missing parents, like mkdir -p. Returns false when a
// part of the path cannot be made a directory.
bool MakeDirectories(const std::string& path) {
  for (std::size_t end = path.find('/', 1); end != std::string::npos;
       end = path.find('/', end + 1)) {
    const std::string parent = path.substr(0, end);
    if (mkdir(parent.c_str(), 0777) != 0 && errno != EEXIST) {
      return false;
    }
  }
  if (mkdir(path.c_str(), 0777) != 0 && errno != EEXIST) {
    return false;
  }
  struct stat info = {};
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

struct OutputFile {
  std::string path;
  std::string text;
};

// Writes text to path + ".tmp". Returns false, and removes the partial file,
// if the write fails.
bool WriteTemporary(const OutputFile& output) {
  const std::string temporary = output.path + ".tmp";
  {
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    file << output.text;
    file.close();
    if (file) {
      return true;
    }
  }
  std::remove(temporary.c_str());
  return false;
}

// The files of one run and the directory they go to.
struct OutputBatch {
  std::string directory;
  std::vector<OutputFile> files;
};

// Writes each file next to its final name first and renames them only when
// all are complete, so a failed run leaves the old outputs as they were.
// Returns the reason for a failure, or std::nullopt when every file is
// written.
std::optional<std::string> WriteOutputs(const OutputBatch& batch) {
  const std::string& out_dir = batch.directory;
  const std::vector<OutputFile>& files = batch.files;
  if (!MakeDirectories(out_dir)) {
    return "cannot create the output directory " + out_dir;
  }
  const auto remove_temporaries = [&files] {
    for (const OutputFile& file : files) {
      std::remove((file.path + ".tmp").c_str());
    }
  };
  for (const OutputFile& file : files) {
    if (!WriteTemporary(file)) {
      remove_temporaries();
      return "cannot write " + file.path + ".tmp";
    }
  }
  const bool renamed =
      std::ranges::all_of(files, [](const OutputFile& file) {
        return std::rename((file.path + ".tmp").c_str(), file.path.c_str()) ==
               0;
      });
  if (!renamed) {
    remove_temporaries();
    return "cannot replace the outputs in " + out_dir;
  }
  return std::nullopt;
}

// One seed's numbers. An empty optional means the value could not be
// computed for this seed, which counts as a failed criterion.
struct SeedRun {
  std::uint64_t seed = 0;
  std::optional<double> raw_gnss_rmse;  // Criterion 2, metres.
  std::optional<double> filter_rmse;    // Criterion 2, metres.
  std::optional<double> reduction;      // Criterion 2, fraction below raw.
  std::optional<Reacquisition> reacquisition;  // Criterion 3.
};


// RunEkf() reports an invalid configuration or a numeric failure by
// throwing. This catches it and says which run failed.
std::optional<std::vector<EkfEstimate>> RunFilter(
    std::uint64_t seed, std::string_view label,
    std::span<const PlanarImu> imu, std::span<const GnssFix> gnss,
    const EkfConfig& config) {
  try {
    return RunEkf(imu, gnss, config);
  } catch (const std::exception& error) {
    Report("seed " + std::to_string(seed) + ", " + std::string(label) +
           ": the EKF failed: " + error.what());
    return std::nullopt;
  }
}

// Measures one seed. Both criteria use the same noise, so the only change
// between the drives is the outage.
std::optional<SeedRun> MeasureSeed(std::uint64_t seed,
                                    const SyntheticConfig& base,
                                    const EkfConfig& ekf_base) {
  SyntheticConfig with_outage = base;
  with_outage.seed = seed;
  SyntheticConfig without_outage = with_outage;
  without_outage.outage_duration = 0.0;

  const std::optional<Scenario> clear =
      GenerateScenario(DemoDrive(), without_outage);
  const std::optional<Scenario> outage =
      GenerateScenario(DemoDrive(), with_outage);
  // The two drives must differ only in the outage. Their IMU data and their
  // reference come from their own streams, so they have to be identical.
  const auto same_imu = [](const PlanarImu& x, const PlanarImu& y) {
    return x.t == y.t && x.ax == y.ax && x.wz == y.wz;
  };
  const auto same_reference = [](const ReferenceSample& x,
                                 const ReferenceSample& y) {
    return x.t == y.t && x.state.east == y.state.east &&
           x.state.north == y.state.north && x.state.yaw == y.state.yaw &&
           x.state.v == y.state.v;
  };
  if (!clear.has_value() || !outage.has_value() ||
      !std::ranges::equal(clear->imu, outage->imu, same_imu) ||
      !std::ranges::equal(clear->reference, outage->reference,
                          same_reference)) {
    Report("seed " + std::to_string(seed) +
           ": cannot build the synthetic drive");
    return std::nullopt;
  }

  // Both filters start where the drive starts, as kitti_run does.
  EkfConfig ekf = ekf_base;
  ekf.initial_state = clear->reference.front().state;
  const TimeWindow window = {
      .start = with_outage.outage_start,
      .end = with_outage.outage_start + with_outage.outage_duration,
  };

  const std::optional<std::vector<EkfEstimate>> clear_run =
      RunFilter(seed, "drive without outage", clear->imu, clear->gnss, ekf);
  const std::optional<std::vector<EkfEstimate>> outage_run =
      RunFilter(seed, "drive with outage", outage->imu, outage->gnss, ekf);
  // Without fixes the two drives give the same IMU-only run.
  const std::optional<std::vector<EkfEstimate>> imu_only = RunFilter(
      seed, "IMU only", clear->imu, std::span<const GnssFix>{}, ekf);
  if (!clear_run.has_value() || !outage_run.has_value() ||
      !imu_only.has_value()) {
    return std::nullopt;
  }

  // ComputeFusionMetrics() needs both a full and an outage run. Each drive is
  // measured with its own run, so the other field repeats it. Each criterion
  // reads only the numbers that come from its own run.
  const std::optional<FusionMetrics> clear_metrics = ComputeFusionMetrics({
      .reference = clear->reference,
      .gnss = clear->gnss,
      .full = *clear_run,
      .outage = *clear_run,
      .imu_only = *imu_only,
      .outage_window = window,
  });
  const std::optional<FusionMetrics> outage_metrics = ComputeFusionMetrics({
      .reference = outage->reference,
      .gnss = outage->gnss,
      .full = *outage_run,
      .outage = *outage_run,
      .imu_only = *imu_only,
      .outage_window = window,
  });
  if (!clear_metrics.has_value() || !outage_metrics.has_value()) {
    Report("seed " + std::to_string(seed) +
           ": the estimates do not match the reference");
    return std::nullopt;
  }

  return SeedRun{
      .seed = seed,
      .raw_gnss_rmse = clear_metrics->raw_gnss_rmse,
      .filter_rmse = clear_metrics->filter_rmse,
      .reduction = clear_metrics->filter_rmse_reduction,
      .reacquisition = outage_metrics->reacquisition,
  };
}

// One value per seed, possibly missing.
using SeedValues = std::vector<std::optional<double>>;

// One criterion over all seeds.
struct Verdict {
  std::string_view id;      // "2" or "3".
  std::string_view claim;   // What the criterion says.
  double target = 0.0;      // Met when a value is at least the target.
  SeedValues values;  // One per seed.
  std::optional<double> median;
  std::optional<double> min;
  std::optional<double> max;
  std::size_t seeds_met = 0;
  bool met = false;  // Every seed meets the target.
};

// The middle value of a sorted, non-empty list.
double Median(const std::vector<double>& sorted) {
  const std::size_t middle = sorted.size() / 2;
  if (sorted.size() % 2 == 1) {
    return sorted[middle];
  }
  return (sorted[middle - 1] + sorted[middle]) / 2.0;
}

// What a criterion is called and what it requires.
struct CriterionSpec {
  std::string_view id;
  std::string_view claim;
  double target = 0.0;
};

Verdict MakeVerdict(const CriterionSpec& spec,
                    SeedValues values) {
  Verdict verdict = {
      .id = spec.id,
      .claim = spec.claim,
      .target = spec.target,
      .values = std::move(values),
      .median = std::nullopt,
      .min = std::nullopt,
      .max = std::nullopt,
      .seeds_met = 0,
      .met = false,
  };
  std::vector<double> present;
  for (const std::optional<double>& value : verdict.values) {
    if (value.has_value()) {
      present.push_back(*value);
      if (*value >= spec.target) {
        ++verdict.seeds_met;
      }
    }
  }
  if (!present.empty()) {
    std::ranges::sort(present);
    verdict.min = present.front();
    verdict.max = present.back();
    verdict.median = Median(present);
  }
  // A seed without a value does not meet the target.
  verdict.met = !verdict.values.empty() &&
                verdict.seeds_met == verdict.values.size();
  return verdict;
}

// A fixed-point number with Digits decimals, for the verdict lines.
template <int Digits>
std::string FormatFixed(double value) {
  std::array<char, 64> buffer = {};
  std::snprintf(buffer.data(), buffer.size(), "%.*f", Digits, value);
  return buffer.data();
}

std::string Met(const Verdict& verdict) {
  return verdict.met ? "met" : "not met";
}

// One verdict line for stderr, in the units of the criterion.
std::string Describe(const Verdict& verdict, std::size_t seeds) {
  const bool percent = verdict.id == "2";
  const auto shown = [percent](double value) {
    return percent ? FormatFixed<1>(value * 100.0) + "%"
                   : FormatFixed<2>(value) + "x";
  };
  std::string line = "criterion " + std::string(verdict.id) + " (" +
                     std::string(verdict.claim) + "): ";
  if (verdict.median.has_value() && verdict.min.has_value() &&
      verdict.max.has_value()) {
    line += "median " + shown(*verdict.median) + ", min " +
            shown(*verdict.min) + ", max " + shown(*verdict.max) + ", ";
  } else {
    line += "no value for any seed, ";
  }
  line += std::to_string(verdict.seeds_met) + " of " +
          std::to_string(seeds) + " seeds: " + Met(verdict) + " (target " +
          (percent ? FormatFixed<0>(verdict.target * 100.0) + "% or more"
                   : FormatFixed<0>(verdict.target) + "x or more") +
          ")";
  return line;
}

void AppendVerdictJson(const Verdict& verdict, std::string& text) {
  text += R"(    ")" + std::string(verdict.id) + R"(": {"target": )";
  AppendJsonNumber(verdict.target, text);
  text += ", \"median\": ";
  AppendJsonNumber(verdict.median, text);
  text += ", \"min\": ";
  AppendJsonNumber(verdict.min, text);
  text += ", \"max\": ";
  AppendJsonNumber(verdict.max, text);
  text += ", \"seeds_met\": " + std::to_string(verdict.seeds_met);
  text += R"(, "verdict": ")" + Met(verdict) + R"("})";
}

std::string AccuracyJson(const std::vector<SeedRun>& runs,
                         const SyntheticConfig& base,
                         const std::vector<Verdict>& verdicts,
                         const EkfConfig& ekf) {
  std::string text = "{\n";
  text += "  \"schema\": 1,\n";
  text += "  \"producer\": \"accuracy\",\n";
  text += "  \"seeds\": " + std::to_string(runs.size()) + ",\n";
  text += R"(  "scenario": {"drive": "DemoDrive", "outage_start": )";
  AppendJsonNumber(base.outage_start, text);
  text += ", \"outage_duration\": ";
  AppendJsonNumber(base.outage_duration, text);
  text += ", \"gnss_period\": ";
  AppendJsonNumber(base.gnss_period, text);
  text += ", \"gnss_noise\": ";
  AppendJsonNumber(base.gnss_noise, text);
  text += "},\n";
  text += R"(  "ekf": {"initial_position_std": )";
  AppendJsonNumber(ekf.initial_position_std, text);
  text += ", \"initial_yaw_std\": ";
  AppendJsonNumber(ekf.initial_yaw_std, text);
  text += ", \"initial_speed_std\": ";
  AppendJsonNumber(ekf.initial_speed_std, text);
  text += ", \"accel_noise_density\": ";
  AppendJsonNumber(ekf.accel_noise_density, text);
  text += ", \"gyro_noise_density\": ";
  AppendJsonNumber(ekf.gyro_noise_density, text);
  text += ", \"gnss_position_std\": ";
  AppendJsonNumber(ekf.gnss_position_std, text);
  text += "},\n";

  text += "  \"runs\": [\n";
  for (std::size_t i = 0; i < runs.size(); ++i) {
    const SeedRun& run = runs[i];
    text += "    {\"seed\": " + std::to_string(run.seed);
    text += R"(, "criterion2": {"raw_gnss_rmse": )";
    AppendJsonNumber(run.raw_gnss_rmse, text);
    text += ", \"filter_rmse\": ";
    AppendJsonNumber(run.filter_rmse, text);
    text += ", \"reduction\": ";
    AppendJsonNumber(run.reduction, text);
    text += "}, \"criterion3\": ";
    if (run.reacquisition.has_value()) {
      const Reacquisition& reacquisition = *run.reacquisition;
      text += "{\"t\": ";
      AppendJsonNumber(reacquisition.t, text);
      text += ", \"filter_error\": ";
      AppendJsonNumber(reacquisition.filter_error, text);
      text += ", \"imu_only_error\": ";
      AppendJsonNumber(reacquisition.imu_only_error, text);
      text += ", \"ratio\": ";
      AppendJsonNumber(reacquisition.ratio, text);
      text += "}";
    } else {
      text += "null";
    }
    text += i + 1 == runs.size() ? "}\n" : "},\n";
  }
  text += "  ],\n";

  text += "  \"criteria\": {\n";
  for (std::size_t i = 0; i < verdicts.size(); ++i) {
    AppendVerdictJson(verdicts[i], text);
    text += i + 1 == verdicts.size() ? "\n" : ",\n";
  }
  text += "  }\n}\n";
  return text;
}

int Run(const Options& options) {
  const SyntheticConfig base;
  const EkfConfig ekf;
  std::vector<SeedRun> runs;
  runs.reserve(options.seeds);
  for (std::size_t seed = 1; seed <= options.seeds; ++seed) {
    const std::optional<SeedRun> run = MeasureSeed(seed, base, ekf);
    if (!run.has_value()) {
      return kExitFailure;
    }
    runs.push_back(*run);
  }

  SeedValues reductions;
  SeedValues ratios;
  for (const SeedRun& run : runs) {
    reductions.push_back(run.reduction);
    ratios.push_back(run.reacquisition.has_value()
                         ? std::optional<double>(run.reacquisition->ratio)
                         : std::nullopt);
  }
  const std::vector<Verdict> verdicts = {
      MakeVerdict({.id = "2",
                   .claim = "filter RMSE below raw GNSS on the whole route",
                   .target = kCriterion2MinReduction},
                  std::move(reductions)),
      MakeVerdict({.id = "3",
                   .claim = "error ratio at reacquisition after the outage",
                   .target = kCriterion3MinRatio},
                  std::move(ratios)),
  };

  const std::string path = options.out_dir + "/" + std::string(kMetricsName);
  const std::optional<std::string> failure = WriteOutputs({
      .directory = options.out_dir,
      .files = {{.path = path, .text = AccuracyJson(runs, base, verdicts, ekf)}},
  });
  if (failure.has_value()) {
    Report(*failure);
    return kExitFailure;
  }
  for (const Verdict& verdict : verdicts) {
    Report(Describe(verdict, runs.size()));
  }
  Report("wrote " + path);
  return kExitOk;
}

int Main(std::span<char* const> args) {
  const std::optional<Options> options = ParseArguments(args);
  if (!options.has_value()) {
    std::cerr << kUsage;
    return kExitUsage;
  }
  if (options->help) {
    std::cout << kUsage;
    return kExitOk;
  }
  return Run(*options);
}

}  // namespace
}  // namespace sensor_fusion

int main(int argc, char** argv) {
  // Main() reports its own failures. This catches what still escapes it,
  // such as std::bad_alloc, so the program ends with a failure instead of
  // std::terminate().
  try {
    return sensor_fusion::Main(
        std::span<char* const>(argv, static_cast<std::size_t>(argc)));
  } catch (...) {
    std::fputs("accuracy: internal error\n", stderr);
    return 1;
  }
}
