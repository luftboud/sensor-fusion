// Runs the EKF on a KITTI oxts drive with the GNSS fixes that
// foreign_impl/ref_ekf.py wrote, once with every fix and once with the
// outage, and writes both trajectories as CSV for the comparison with
// filterpy, with the accuracy metrics as JSON:
//
//   kitti_run --oxts DIR --gnss FILE --gnss-outage FILE [--out DIR]
//
// See docs/kitti-scenario.md#running-the-comparison.

#include <sys/stat.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cerrno>
#include <charconv>
#include <cstddef>
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
#include "sensor_fusion/io/gnss/gnss_csv.h"
#include "sensor_fusion/io/imu/imu.h"
#include "sensor_fusion/io/kitti/kitti_scenario.h"
#include "sensor_fusion/io/text.h"

namespace sensor_fusion {
namespace {

constexpr int kExitOk = 0;
constexpr int kExitDataError = 1;
constexpr int kExitUsage = 2;

constexpr std::string_view kProgram = "kitti_run";
constexpr std::string_view kEstimatesHeader =
    "t,east,north,yaw,v,std_east,std_north,updated";
constexpr std::string_view kFullName = "kitti_estimates.csv";
constexpr std::string_view kOutageName = "kitti_estimates_outage.csv";
constexpr std::string_view kMetricsName = "kitti_metrics.json";
// The layout of kitti_metrics.json. Increase it when a key is renamed,
// removed or changes meaning; adding a key keeps the number.
constexpr std::size_t kMetricsSchema = 1;

constexpr std::string_view kUsage =
    "usage: kitti_run --oxts DIR --gnss FILE --gnss-outage FILE [--out DIR]\n"
    "                 [--initial-position-std M] [--initial-yaw-std RAD]\n"
    "                 [--initial-speed-std M/S] [--accel-noise-density X]\n"
    "                 [--gyro-noise-density X] [--gnss-position-std M]\n"
    "                 [--outage-start S] [--outage-length S]\n"
    "\n"
    "  --oxts DIR         KITTI oxts directory (timestamps.txt and data/)\n"
    "  --gnss FILE        GNSS fixes for the whole drive, t,east,north\n"
    "  --gnss-outage FILE the same fixes with the outage window cut out\n"
    "  --out DIR          output directory, default build/report/kitti\n"
    "  --outage-start S   start of the outage window, default 15 s\n"
    "  --outage-length S  length of the outage window, default 15 s\n"
    "\n"
    "The filter flags default to EkfConfig. The initial state is the first\n"
    "reference sample: east 0, north 0, the OXTS yaw and the forward speed.\n"
    "The outage window must match the one ref_ekf.py cut from the fixes:\n"
    "the outage file must hold every fix outside it and none inside.\n"
    "Writes kitti_estimates.csv, kitti_estimates_outage.csv and\n"
    "kitti_metrics.json.\n"
    "Exit status: 0 done, 1 bad input data or write error, 2 bad arguments.\n";

struct Options {
  std::string oxts_dir;
  std::string gnss_path;
  std::string gnss_outage_path;
  std::string out_dir = "build/report/kitti";
  EkfConfig config;
  // PARAMS["outage_start"] and PARAMS["outage_length"] in ref_ekf.py.
  double outage_start = 15.0;
  double outage_length = 15.0;
  bool help = false;
};

struct NumberFlag {
  std::string_view name;
  double EkfConfig::*field;
};

constexpr std::array<NumberFlag, 6> kNumberFlags = {{
    {.name = "--initial-position-std",
     .field = &EkfConfig::initial_position_std},
    {.name = "--initial-yaw-std", .field = &EkfConfig::initial_yaw_std},
    {.name = "--initial-speed-std", .field = &EkfConfig::initial_speed_std},
    {.name = "--accel-noise-density",
     .field = &EkfConfig::accel_noise_density},
    {.name = "--gyro-noise-density", .field = &EkfConfig::gyro_noise_density},
    {.name = "--gnss-position-std", .field = &EkfConfig::gnss_position_std},
}};

// Prints "kitti_run: message" to stderr.
void Report(std::string_view message) {
  std::cerr << kProgram << ": " << message << '\n';
}

// Sets one option from a flag and its value. Returns false, after printing
// why, when the flag is unknown or the value does not fit it.
bool SetOption(std::string_view flag, std::string_view value,
               Options& options) {
  std::string* path = nullptr;
  if (flag == "--oxts") {
    path = &options.oxts_dir;
  } else if (flag == "--gnss") {
    path = &options.gnss_path;
  } else if (flag == "--gnss-outage") {
    path = &options.gnss_outage_path;
  } else if (flag == "--out") {
    path = &options.out_dir;
  }
  if (path != nullptr) {
    if (value.empty()) {
      Report(std::string(flag) + " needs a non-empty path");
      return false;
    }
    *path = std::string(value);
    return true;
  }

  if (flag == "--outage-start" || flag == "--outage-length") {
    const std::optional<double> parsed = internal::ParseDouble(value);
    const bool is_length = flag == "--outage-length";
    if (!parsed.has_value() || (is_length && *parsed < 0.0)) {
      Report(std::string(flag) + " needs a finite" +
             (is_length ? " nonnegative" : "") + " number of seconds, got '" +
             std::string(value) + "'");
      return false;
    }
    (is_length ? options.outage_length : options.outage_start) = *parsed;
    return true;
  }

  for (const NumberFlag& number : kNumberFlags) {
    if (flag == number.name) {
      const std::optional<double> parsed = internal::ParseDouble(value);
      if (!parsed.has_value()) {
        Report(std::string(flag) + " needs a finite number, got '" +
               std::string(value) + "'");
        return false;
      }
      options.config.*number.field = *parsed;
      return true;
    }
  }

  Report("unknown flag '" + std::string(flag) + "'");
  return false;
}

// Reads "--flag value" pairs. Returns std::nullopt, after printing why, on
// a bad command line.
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
    ++i;
    if (!SetOption(flag, args[i], options)) {
      return std::nullopt;
    }
  }

  if (options.oxts_dir.empty() || options.gnss_path.empty() ||
      options.gnss_outage_path.empty()) {
    Report("--oxts, --gnss and --gnss-outage are required");
    return std::nullopt;
  }
  return options;
}

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

std::string EstimatesCsv(const std::vector<EkfEstimate>& estimates) {
  std::string text(kEstimatesHeader);
  text += '\n';
  for (const EkfEstimate& estimate : estimates) {
    for (const double value :
         {estimate.t, estimate.state.east, estimate.state.north,
          estimate.state.yaw, estimate.state.v, estimate.std_east,
          estimate.std_north}) {
      AppendNumber(value, text);
      text += ',';
    }
    text += estimate.updated ? "1\n" : "0\n";
  }
  return text;
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

// Writes text to path + ".tmp". Returns false, and removes the partial file,
// if the write fails.
bool WriteTemporary(const std::string& path, std::string_view text) {
  const std::string temporary = path + ".tmp";
  {
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    file << text;
    file.close();
    if (file) {
      return true;
    }
  }
  std::remove(temporary.c_str());
  return false;
}

// Appends value as a JSON string: quoted, with quotes, backslashes and
// control characters escaped.
void AppendJsonString(std::string_view value, std::string& text) {
  constexpr std::string_view kHex = "0123456789abcdef";
  text += '"';
  for (const char c : value) {
    const auto code = static_cast<unsigned char>(c);
    if (c == '"' || c == '\\') {
      text += '\\';
      text += c;
    } else if (code < 0x20) {
      text += "\\u00";
      text += kHex[code >> 4U];
      text += kHex[code & 0xFU];
    } else {
      text += c;
    }
  }
  text += '"';
}

// Builds indented JSON one key or value at a time. Objects nest; every key
// is followed by exactly one value or object.
class JsonWriter {
 public:
  void BeginObject() {
    text_ += '{';
    first_.push_back(true);
  }
  void EndObject() {
    first_.pop_back();
    text_ += '\n';
    Indent();
    text_ += '}';
  }
  void Key(std::string_view name) {
    if (!first_.back()) {
      text_ += ',';
    }
    first_.back() = false;
    text_ += '\n';
    Indent();
    AppendJsonString(name, text_);
    text_ += ": ";
  }
  void String(std::string_view value) { AppendJsonString(value, text_); }
  void Integer(std::size_t value) { text_ += std::to_string(value); }
  // null for a missing or non-finite value.
  void Number(std::optional<double> value) {
    if (value.has_value() && std::isfinite(*value)) {
      AppendNumber(*value, text_);
    } else {
      text_ += "null";
    }
  }
  void Null() { text_ += "null"; }
  [[nodiscard]] std::string Finish() const { return text_ + '\n'; }

 private:
  void Indent() { text_.append(2 * first_.size(), ' '); }

  std::string text_;
  std::vector<bool> first_;  // Per open object: no key written yet.
};

// "--initial-position-std" as the JSON key "initial_position_std".
std::string ConfigKey(std::string_view flag) {
  std::string key(flag.substr(2));
  std::ranges::replace(key, '-', '_');
  return key;
}

// What kitti_metrics.json describes besides the metrics.
struct MetricsContext {
  const Options& options;
  std::size_t steps = 0;
  double duration = 0.0;  // Last step time minus the first (s).
  std::size_t fixes = 0;
  std::size_t outage_fixes = 0;
};

std::string MetricsJson(const FusionMetrics& metrics,
                        const MetricsContext& context) {
  JsonWriter json;
  json.BeginObject();
  json.Key("schema");
  json.Integer(kMetricsSchema);
  json.Key("producer");
  json.String(kProgram);

  json.Key("dataset");
  json.BeginObject();
  json.Key("oxts");
  json.String(context.options.oxts_dir);
  json.Key("steps");
  json.Integer(context.steps);
  json.Key("duration_s");
  json.Number(context.duration);
  json.EndObject();

  json.Key("gnss");
  json.BeginObject();
  json.Key("full_fixes");
  json.Integer(context.fixes);
  json.Key("outage_fixes");
  json.Integer(context.outage_fixes);
  json.Key("outage_start_s");
  json.Number(context.options.outage_start);
  json.Key("outage_length_s");
  json.Number(context.options.outage_length);
  json.EndObject();

  json.Key("config");
  json.BeginObject();
  for (const NumberFlag& number : kNumberFlags) {
    json.Key(ConfigKey(number.name));
    json.Number(context.options.config.*number.field);
  }
  json.EndObject();

  json.Key("metrics");
  json.BeginObject();
  json.Key("raw_gnss_rmse_m");
  json.Number(metrics.raw_gnss_rmse);
  json.Key("filter_rmse_m");
  json.Number(metrics.filter_rmse);
  json.Key("filter_rmse_reduction");
  json.Number(metrics.filter_rmse_reduction);
  json.Key("filter_outage_rmse_route_m");
  json.Number(metrics.outage_rmse);
  json.Key("filter_outage_rmse_window_m");
  json.Number(metrics.outage_window_rmse);
  json.Key("imu_only_window_rmse_m");
  json.Number(metrics.imu_only_window_rmse);
  json.Key("reacquisition");
  if (metrics.reacquisition.has_value()) {
    const Reacquisition& reacquisition = *metrics.reacquisition;
    json.BeginObject();
    json.Key("t_s");
    json.Number(reacquisition.t);
    json.Key("filter_error_m");
    json.Number(reacquisition.filter_error);
    json.Key("imu_only_error_m");
    json.Number(reacquisition.imu_only_error);
    json.Key("ratio");
    json.Number(reacquisition.ratio);
    json.EndObject();
  } else {
    json.Null();
  }
  json.EndObject();

  json.EndObject();
  return json.Finish();
}

struct OutputFile {
  std::string path;
  std::string text;
};

// Writes each file next to its final name first and renames them only when
// all are complete, so a failed run leaves the old outputs as they were.
bool WriteOutputs(const std::string& out_dir,
                  const std::vector<OutputFile>& files) {
  if (!MakeDirectories(out_dir)) {
    Report("cannot create the output directory " + out_dir);
    return false;
  }
  const auto remove_temporaries = [&files] {
    for (const OutputFile& file : files) {
      std::remove((file.path + ".tmp").c_str());
    }
  };
  for (const OutputFile& file : files) {
    if (!WriteTemporary(file.path, file.text)) {
      remove_temporaries();
      Report("cannot write " + file.path + ".tmp");
      return false;
    }
  }
  const bool renamed =
      std::ranges::all_of(files, [](const OutputFile& file) {
        return std::rename((file.path + ".tmp").c_str(), file.path.c_str()) ==
               0;
      });
  if (!renamed) {
    remove_temporaries();
    Report("cannot replace the outputs in " + out_dir);
  }
  return renamed;
}

std::optional<std::vector<GnssFix>> ReadFixes(const std::string& path) {
  const GnssCsvResult result = ReadGnssCsv(path);
  if (!result.ok()) {
    std::string message = path + ": " + std::string(ToString(result.error));
    if (result.line != 0) {
      message += " (line " + std::to_string(result.line) + ")";
    }
    Report(message);
    return std::nullopt;
  }
  return result.fixes;
}

// AppendNumber() as a string, for messages.
std::string NumberText(double value) {
  std::string text;
  AppendNumber(value, text);
  return text;
}

// The two GNSS inputs, named so that they cannot be passed swapped.
struct GnssFiles {
  std::span<const GnssFix> full;    // --gnss
  std::span<const GnssFix> outage;  // --gnss-outage
};

// The outage file must hold exactly the fixes of the full file outside the
// window, in the same order, as cut_outage() in ref_ekf.py writes it. Any
// other file, such as the full file itself or one cut with another window,
// would give metrics for an outage that did not happen. Returns why the two
// files disagree, or std::nullopt when they agree.
std::optional<std::string> OutageMismatch(const GnssFiles& gnss,
                                          const TimeWindow& window) {
  const std::span<const GnssFix> outage_fixes = gnss.outage;
  std::size_t next = 0;  // The next outage fix to match.
  for (const GnssFix& fix : gnss.full) {
    if (window.Contains(fix.t)) {
      continue;
    }
    if (next == outage_fixes.size()) {
      return "it lacks the fix at t = " + NumberText(fix.t) + " s";
    }
    const GnssFix& kept = outage_fixes[next];
    if (kept.t != fix.t || kept.east != fix.east || kept.north != fix.north) {
      break;
    }
    ++next;
  }
  if (next == outage_fixes.size()) {
    return std::nullopt;
  }
  const GnssFix& extra = outage_fixes[next];
  if (window.Contains(extra.t)) {
    return "it keeps the fix at t = " + NumberText(extra.t) +
           " s inside the window";
  }
  return "its fix at t = " + NumberText(extra.t) +
         " s is not the next fix of the full file";
}

// RunEkf() reports an invalid configuration or a numeric failure by
// throwing. This catches it and prints why.
std::optional<std::vector<EkfEstimate>> RunFilter(
    const Scenario& scenario, const std::vector<GnssFix>& fixes,
    const EkfConfig& config, std::string_view label) {
  try {
    return RunEkf(scenario.imu, fixes, config);
  } catch (const std::exception& error) {
    Report("EKF failed on " + std::string(label) + ": " + error.what());
    return std::nullopt;
  }
}

int Run(const Options& options) {
  // RunEkf() checks the configuration before the data, so a run on no data
  // tells a bad flag value apart from bad input files.
  try {
    static_cast<void>(RunEkf({}, {}, options.config));
  } catch (const std::exception& error) {
    Report(std::string("invalid filter parameters: ") + error.what());
    return kExitUsage;
  }

  const KittiReadResult drive = ReadKittiOxts(options.oxts_dir);
  if (!drive.ok()) {
    Report(options.oxts_dir + ": " + std::string(ToString(drive.error)) +
           " (sample " + std::to_string(drive.index) + ")");
    return kExitDataError;
  }
  const Scenario scenario = KittiScenario(drive);

  const std::optional<std::vector<GnssFix>> fixes =
      ReadFixes(options.gnss_path);
  const std::optional<std::vector<GnssFix>> outage_fixes =
      ReadFixes(options.gnss_outage_path);
  if (!fixes.has_value() || !outage_fixes.has_value()) {
    return kExitDataError;
  }
  const TimeWindow window = {
      .start = options.outage_start,
      .end = options.outage_start + options.outage_length,
  };
  if (const std::optional<std::string> mismatch =
          OutageMismatch({.full = *fixes, .outage = *outage_fixes}, window)) {
    Report(options.gnss_outage_path + " is not " + options.gnss_path +
           " without the outage window [" + NumberText(window.start) + ", " +
           NumberText(window.end) + ") s: " + *mismatch);
    return kExitDataError;
  }

  // Both filters start where ref_ekf.py starts: at the first reference
  // sample, which is (0, 0, yaw0, vf0).
  EkfConfig config = options.config;
  config.initial_state = scenario.reference.front().state;

  const std::optional<std::vector<EkfEstimate>> full =
      RunFilter(scenario, *fixes, config, options.gnss_path);
  const std::optional<std::vector<EkfEstimate>> outage =
      RunFilter(scenario, *outage_fixes, config, options.gnss_outage_path);
  const std::optional<std::vector<EkfEstimate>> imu_only =
      RunFilter(scenario, {}, config, "the IMU-only run");
  if (!full.has_value() || !outage.has_value() || !imu_only.has_value()) {
    return kExitDataError;
  }

  const std::optional<FusionMetrics> metrics = ComputeFusionMetrics({
      .reference = scenario.reference,
      .gnss = *fixes,
      .full = *full,
      .outage = *outage,
      .imu_only = *imu_only,
      .outage_window = window,
  });
  if (!metrics.has_value()) {
    Report("the estimates do not match the reference trajectory");
    return kExitDataError;
  }

  const std::string prefix = options.out_dir + "/";
  const MetricsContext context = {
      .options = options,
      .steps = full->size(),
      .duration = scenario.reference.back().t - scenario.reference.front().t,
      .fixes = fixes->size(),
      .outage_fixes = outage_fixes->size(),
  };
  if (!WriteOutputs(
          options.out_dir,
          {{.path = prefix + std::string(kFullName),
            .text = EstimatesCsv(*full)},
           {.path = prefix + std::string(kOutageName),
            .text = EstimatesCsv(*outage)},
           {.path = prefix + std::string(kMetricsName),
            .text = MetricsJson(*metrics, context)}})) {
    return kExitDataError;
  }
  Report(std::to_string(full->size()) + " steps, " +
         std::to_string(fixes->size()) + " and " +
         std::to_string(outage_fixes->size()) + " fixes, written to " +
         options.out_dir);
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
  // such as std::bad_alloc, so the program ends with a data error instead
  // of std::terminate().
  try {
    return sensor_fusion::Main(
        std::span<char* const>(argv, static_cast<std::size_t>(argc)));
  } catch (...) {
    std::fputs("kitti_run: internal error\n", stderr);
    return 1;
  }
}
