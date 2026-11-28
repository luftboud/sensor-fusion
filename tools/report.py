"""
    Builds the project, runs every check that exists, and assembles the report
    against the success criteria of docs/project.md.

    This script computes no criterion formula. The numbers come from the C++
    programs (apps/report/accuracy.cpp, apps/report/kitti_run.cpp, through
    include/sensor_fusion/ekf/metrics.h) and from foreign_impl/ref_ekf.py. The
    targets come from include/sensor_fusion/ekf/targets.h. The script runs the
    programs, reads their JSON, compares the numbers with the targets and
    prints them. A result that did not run is never shown as met.

    Usage, from anywhere:
        python tools/report.py [--build-dir DIR] [--oxts DIR] [--seeds N]
                               [--python EXE] [--skip-build] [--strict]
                               [--no-docs] [--plain]

    With rich installed (tools/requirements.txt) the terminal view has
    spinners, a panel and a coloured table; otherwise, or with --plain, it is
    plain text. NO_COLOR and a non-terminal stdout turn colours off.

    Writes docs/report/summary.md, docs/report/accuracy.md and
    docs/report/kitti.md, and prints one table. The raw JSON stays in
    <build>/report/. Exit status: 0 the report was made, 1 the build failed or
    the script itself failed; with --strict, also 1 when a criterion is not met.
"""

import argparse
import contextlib
import dataclasses
import datetime
import json
import os
import pathlib
import platform
import re
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ElementTree

try:  # Optional: a nicer terminal view. Without it the output is plain.
    import rich.console
    import rich.markup
except ImportError:
    rich = None

ROOT = pathlib.Path(__file__).resolve().parents[1]
TARGETS_H = ROOT / "include/sensor_fusion/ekf/targets.h"
DOCS_REPORT = ROOT / "docs/report"
DEFAULT_OXTS = "data/kitti/2011_09_26/2011_09_26_drive_0009_sync/oxts"
STDERR_TAIL_LINES = 15

# Statuses. Only MET is green.
MET = "met"
NOT_MET = "not met"
NOT_RUN = "not run"
NOT_MEASURED = "not measured"
STALE = "stale"
PARTIAL = "partial"

COLOURS = {MET: "32", PARTIAL: "33"}  # Everything else is red (31).


# --------------------------------------------------------------------------
# Running steps

@dataclasses.dataclass
class Step:
    name: str
    command: list
    status: str = NOT_RUN  # "ok", "failed" or NOT_RUN.
    exit_code: int | None = None
    seconds: float = 0.0
    reason: str = ""  # Why it did not run, or the tail of its stderr.

    @property
    def ok(self):
        return self.status == "ok"


def run_step(steps, name, command, quiet=True):
    """Runs command from the repository root and records it as a step."""
    # Paths under the repository are passed relative to it, so the outputs
    # and the committed report do not hold this machine's absolute paths.
    prefix = str(ROOT) + os.sep
    parts = [str(part) for part in command]
    parts = [part[len(prefix):] if part.startswith(prefix) else
             "." if part == str(ROOT) else part for part in parts]
    step = Step(name, parts)
    steps.append(step)
    start = time.monotonic()
    with UI.running(step):
        try:
            result = subprocess.run(step.command, cwd=ROOT, text=True,
                                    capture_output=quiet)
        except OSError as error:
            result = None
            step.status = "failed"
            step.reason = str(error)
    step.seconds = time.monotonic() - start
    if result is not None:
        step.exit_code = result.returncode
        step.status = "ok" if result.returncode == 0 else "failed"
        if quiet:
            output = (result.stderr or "") + (result.stdout or "")
            step.reason = "\n".join(
                output.strip().splitlines()[-STDERR_TAIL_LINES:])
    UI.finished(step)
    return step


def skip_step(steps, name, reason):
    step = Step(name, [], NOT_RUN, reason=reason)
    steps.append(step)
    UI.finished(step)
    return step


def read_json(path):
    try:
        return json.loads(pathlib.Path(path).read_text())
    except (OSError, ValueError):
        return None


def remove(path):
    try:
        pathlib.Path(path).unlink()
    except FileNotFoundError:
        pass


# --------------------------------------------------------------------------
# Environment and targets

def command_output(command):
    try:
        result = subprocess.run(command, cwd=ROOT, text=True,
                                capture_output=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def cmake_cache(build_dir):
    values = {}
    try:
        text = (build_dir / "CMakeCache.txt").read_text()
    except OSError:
        return values
    for line in text.splitlines():
        match = re.match(r"^([A-Za-z_]+):[A-Z]+=(.*)$", line)
        if match:
            values[match.group(1)] = match.group(2)
    return values


def python_module_version(python, module):
    return command_output(
        [python, "-c", f"import {module}; print({module}.__version__)"])


def environment(build_dir, python):
    cache = cmake_cache(build_dir)
    compiler = cache.get("CMAKE_CXX_COMPILER", "")
    compiler_version = None
    if compiler:
        version = command_output([compiler, "--version"])
        compiler_version = version.splitlines()[0] if version else None
    commit = command_output(["git", "rev-parse", "--short", "HEAD"])
    dirty = command_output(["git", "status", "--porcelain",
                            "--untracked-files=no"])
    return {
        "commit": (commit or "unknown") + (" (uncommitted changes)" if dirty else ""),
        "branch": command_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]) or "unknown",
        "date": datetime.date.today().isoformat(),
        "platform": f"{platform.system()} {platform.release()} {platform.machine()}",
        "cpu_cores": str(os.cpu_count()),
        "compiler": compiler_version or compiler or "unknown",
        "build_type": cache.get("CMAKE_BUILD_TYPE") or "(none)",
        "cmake": (command_output(["cmake", "--version"]) or "unknown").splitlines()[0],
        "python": command_output([python, "-c", "import platform; print(platform.python_version())"]) or "unknown",
        "filterpy": python_module_version(python, "filterpy") or "not installed",
    }


def read_targets():
    """The kCriterion* constants of targets.h, as floats."""
    text = TARGETS_H.read_text()
    pattern = r"inline\s+constexpr\s+double\s+(kCriterion\w+)\s*=\s*([^;]+);"
    return {name: float(value) for name, value in re.findall(pattern, text)}


# --------------------------------------------------------------------------
# ctest

def read_junit(path):
    """{test name: "passed" | "failed" | "skipped"} from ctest's JUnit file."""
    try:
        tree = ElementTree.parse(path)
    except (OSError, ElementTree.ParseError):
        return {}
    results = {}
    for case in tree.iter("testcase"):
        status = case.get("status")
        if status is None:
            if case.find("failure") is not None or case.find("error") is not None:
                status = "failed"
            elif case.find("skipped") is not None:
                status = "skipped"
            else:
                status = "passed"
        if status == "run":
            status = "passed"
        if status == "fail":
            status = "failed"
        if status == "notrun":
            status = "skipped"
        results[case.get("name")] = status
    return results


# --------------------------------------------------------------------------
# Criteria

@dataclasses.dataclass
class Criterion:
    id: str
    target: str
    measured: str
    status: str
    source: str
    note: str = ""


def percent(value):
    return "n/a" if value is None else f"{value * 100:.1f}%"


def times(value):
    return "n/a" if value is None else f"{value:.2f}x"


def criterion_1(compare, targets):
    bound = targets.get("kCriterion1MaxDifference")
    target = "max |own-filterpy| <= " + (f"{bound:g}" if bound is not None else "?")
    source = "filterpy_compare.json"
    if bound is None:
        return Criterion("1", target, "n/a", NOT_RUN, "targets.h",
                         "kCriterion1MaxDifference is missing from targets.h")
    if compare is None:
        return Criterion("1", target, "n/a", NOT_RUN, source,
                         "the filterpy comparison did not run")
    runs = compare.get("runs", {})
    missing = [name for name in ("full", "outage") if name not in runs]
    errors = [f"{name}: {run['error']}" for name, run in runs.items() if "error" in run]
    if missing or errors:
        note = "; ".join(errors + [f"no {name} run" for name in missing])
        return Criterion("1", target, "n/a", NOT_RUN if missing else NOT_MET,
                         source, note)
    values = [value for run in runs.values()
              for value in run["max_abs_diff"].values()]
    if not values or any(value is None for value in values):
        return Criterion("1", target, "n/a", NOT_MET, source,
                         "a difference is missing")
    largest = max(values)
    status = MET if largest <= bound else NOT_MET
    return Criterion("1", target, f"max {largest:.2g}", status, source,
                     "KITTI drive, both runs, position, yaw, speed and std; "
                     f"filterpy {compare.get('filterpy_version', '?')}")


def criterion_from_accuracy(identifier, accuracy, targets, constant, shown,
                            target_text):
    source = "accuracy.json"
    bound = targets.get(constant)
    target = target_text(bound) if bound is not None else "?"
    if accuracy is None:
        return Criterion(identifier, target, "n/a", NOT_RUN, source,
                         "accuracy did not run")
    if accuracy.get("producer") != "accuracy" or accuracy.get("schema") != 1:
        return Criterion(identifier, target, "n/a", STALE, source,
                         "unknown producer or schema")
    entry = accuracy.get("criteria", {}).get(identifier)
    if entry is None:
        return Criterion(identifier, target, "n/a", NOT_RUN, source,
                         f"criterion {identifier} is not in the file")
    seeds = accuracy.get("seeds", 0)
    measured = (f"median {shown(entry.get('median'))}, "
                f"min {shown(entry.get('min'))}, max {shown(entry.get('max'))}, "
                f"{entry.get('seeds_met', 0)}/{seeds} seeds")
    if bound is None or entry.get("target") != bound:
        return Criterion(identifier, target, measured, STALE, source,
                         f"the file was made with target {entry.get('target')}, "
                         f"targets.h has {bound}; rebuild accuracy")
    status = MET if entry.get("verdict") == "met" else NOT_MET
    return Criterion(identifier, target, measured, status, source,
                     "synthetic drive, every seed must meet the target")


def criterion_6(tests):
    target = "NMEA rejects corrupt; tests pass; covered"
    if not tests:
        return Criterion("6", target, "n/a", NOT_RUN, "ctest.xml",
                         "ctest did not run")
    nmea = tests.get("nmea_tests")
    failed = [name for name, status in tests.items() if status != "passed"]
    passed = len(tests) - len(failed)
    measured = f"nmea_tests {nmea or 'missing'}; {passed}/{len(tests)} tests pass"
    if nmea != "passed" or failed:
        return Criterion("6", target, measured, NOT_MET, "ctest.xml",
                         "failed: " + ", ".join(failed) if failed else "")
    return Criterion("6", target, measured, PARTIAL, "ctest.xml",
                     "coverage not measured yet")


def criterion_7(figure):
    target = "docs, CLI, 3-trajectory figure, report"
    parts = {
        "CLI apps/cli/main.cpp": (ROOT / "apps/cli/main.cpp").is_file(),
        "trajectory figure": figure is not None and figure.is_file(),
        "report docs/report/": True,  # This script writes it.
    }
    present = [name for name, ok in parts.items() if ok]
    missing = [name for name, ok in parts.items() if not ok]
    measured = f"{len(present)}/{len(parts)} parts"
    status = MET if not missing else PARTIAL if present else NOT_MET
    note = "missing: " + ", ".join(missing) if missing else ""
    note += ("; " if note else "") + "performance tables wait for criteria 4-5"
    if status == MET:
        status = PARTIAL  # The report has no performance tables yet.
    return Criterion("7", target, measured, status, "files", note)


def evaluate(targets, accuracy, compare, tests, figure):
    return [
        criterion_1(compare, targets),
        criterion_from_accuracy(
            "2", accuracy, targets, "kCriterion2MinReduction", percent,
            lambda bound: f">= {bound * 100:g}% below raw GNSS"),
        criterion_from_accuracy(
            "3", accuracy, targets, "kCriterion3MinRatio", times,
            lambda bound: f">= {bound:g}x at reacquisition"),
        Criterion("4", "3 optimizations; no hot-path alloc",
                  "n/a", NOT_MEASURED, "-", "benchmarks are not implemented yet"),
        Criterion("5", ">= 70% scaling; vs Eigen",
                  "n/a", NOT_MEASURED, "-", "batched run is not implemented yet"),
        criterion_6(tests),
        criterion_7(figure),
    ]


# --------------------------------------------------------------------------
# Output

def terminal_table(criteria, colour):
    header = ("Criterion", "Target", "Measured", "Status", "Source")
    rows = [(c.id, c.target, c.measured, c.status, c.source) for c in criteria]
    widths = [max(len(row[i]) for row in rows + [header]) for i in range(5)]

    def line(row, status=None):
        cells = [cell.ljust(width) for cell, width in zip(row, widths)]
        if status is not None and colour:
            code = COLOURS.get(status, "31")
            cells[3] = f"\033[{code}m{cells[3]}\033[0m"
        return "  ".join(cells).rstrip()

    lines = [line(header), "  ".join("-" * width for width in widths)]
    lines += [line(row, row[3]) for row in rows]
    notes = [f"  {c.id}: {c.note}" for c in criteria if c.note]
    return "\n".join(lines + ([""] + notes if notes else []))


TITLES = {
    "1": "filterpy match",
    "2": "RMSE vs raw GNSS",
    "3": "outage recovery",
    "4": "optimizations",
    "5": "parallel scaling",
    "6": "NMEA parser",
    "7": "deliverables",
}


class PlainUI:
    """Plain text: progress lines on stderr, one table on stdout."""

    @contextlib.contextmanager
    def running(self, step):
        print(f"[report] {step.name}: {shlex.join(step.command)}",
              file=sys.stderr)
        yield

    def finished(self, step):
        if step.status == NOT_RUN:
            print(f"[report] {step.name}: not run ({step.reason})",
                  file=sys.stderr)
        elif not step.ok:
            print(f"[report] {step.name} failed with exit code "
                  f"{step.exit_code}", file=sys.stderr)

    def final(self, env, criteria, failed_steps, docs):
        print()
        print(f"commit {env['commit']}, {env['compiler']}, "
              f"{env['build_type']}, {env['cpu_cores']} cores")
        colour = sys.stdout.isatty() and "NO_COLOR" not in os.environ
        print(terminal_table(criteria, colour))
        if failed_steps:
            print("\nfailed steps: " + ", ".join(failed_steps))
        if docs:
            print(f"\nwrote {docs}")


class RichUI:
    """The same content through rich: spinners, a panel and a table."""

    BADGES = {
        MET: ("✔ MET", "bold white on green"),
        NOT_MET: ("✘ NOT MET", "bold white on red"),
        NOT_RUN: ("⊘ NOT RUN", "bold red"),
        NOT_MEASURED: ("○ NOT MEASURED", "red"),
        STALE: ("⟳ STALE", "bold magenta"),
        PARTIAL: ("◐ PARTIAL", "bold black on yellow"),
    }
    # One cell of the summary bar per criterion.
    BAR = {MET: ("█", "green"), PARTIAL: ("█", "yellow")}
    COUNT_STYLES = {PARTIAL: "yellow", NOT_MET: "red", STALE: "magenta",
                    NOT_RUN: "red", NOT_MEASURED: "red"}

    def __init__(self):
        self.out = rich.console.Console()
        # Wide terminals get a table that still reads as one block.
        self.out.width = min(self.out.width, 120)
        self.err = rich.console.Console(stderr=True)

    @contextlib.contextmanager
    def running(self, step):
        label = f"[bold]{step.name}[/]  [dim]{shlex.join(step.command)}[/]"
        with self.err.status(label, spinner="dots"):
            yield

    def finished(self, step):
        name = f"{step.name:<17}"
        if step.ok:
            self.err.print(f"  [green]✔[/] {name} [dim]{step.seconds:6.1f} s[/]")
        elif step.status == NOT_RUN:
            self.err.print(f"  [yellow]○[/] {name} [dim]skipped: "
                           f"{rich.markup.escape(step.reason)}[/]")
        else:
            self.err.print(f"  [red]✘[/] {name} [red]exit {step.exit_code}[/]"
                           f" [dim]{step.seconds:6.1f} s[/]")
            for line in step.reason.splitlines()[-5:]:
                self.err.print(f"      [dim]{rich.markup.escape(line)}[/]")

    def final(self, env, criteria, failed_steps, docs):
        from rich import box
        from rich.panel import Panel
        from rich.table import Table
        from rich.text import Text

        facts = Table.grid(padding=(0, 2))
        facts.add_column(style="dim", justify="right")
        facts.add_column()
        facts.add_column(style="dim", justify="right")
        facts.add_column()
        pairs = [("commit", env["commit"]), ("branch", env["branch"]),
                 ("compiler", env["compiler"]), ("build", env["build_type"]),
                 ("platform", env["platform"]), ("cores", env["cpu_cores"]),
                 ("python", env["python"]), ("filterpy", env["filterpy"])]
        for left, right in zip(pairs[0::2], pairs[1::2]):
            facts.add_row(*left, *right)
        self.out.print()
        self.out.print(Panel(
            facts, box=box.ROUNDED, border_style="cyan", padding=(0, 1),
            title="[bold cyan]Sensor Fusion Core[/] · success criteria",
            subtitle=f"[dim]{env['date']}[/]"))

        table = Table(box=box.ROUNDED, header_style="bold cyan",
                      border_style="bright_black", show_lines=True,
                      expand=True)
        # The Source column only fits a wide terminal.
        with_source = self.out.width >= 100
        table.add_column("#", justify="right", style="bold")
        table.add_column("Criterion", ratio=2, min_width=16)
        table.add_column("Measured", ratio=3, min_width=20)
        table.add_column("Status", no_wrap=True)
        if with_source:
            table.add_column("Source", style="dim", no_wrap=True)
        for criterion in criteria:
            text, style = self.BADGES.get(criterion.status,
                                          (criterion.status, "red"))
            measured = Text(criterion.measured)
            if criterion.note:
                measured.append("\n" + criterion.note, style="dim italic")
            title = Text(TITLES.get(criterion.id, ""), style="bold")
            title.append("\n" + criterion.target, style="dim")
            cells = [criterion.id, title, measured,
                     Text(f" {text} ", style=style)]
            if with_source:
                cells.append(criterion.source)
            table.add_row(*cells)
        self.out.print(table)

        counts = {}
        for criterion in criteria:
            counts[criterion.status] = counts.get(criterion.status, 0) + 1
        bar = Text("  ")
        for criterion in criteria:
            glyph, style = self.BAR.get(criterion.status, ("█", "red"))
            bar.append(glyph * 3, style=style)
        bar.append(f"  {counts.get(MET, 0)}/{len(criteria)} met", style="bold")
        for status in (PARTIAL, NOT_MET, STALE, NOT_RUN, NOT_MEASURED):
            if counts.get(status):
                bar.append(f" · {counts[status]} {status}",
                           style=self.COUNT_STYLES[status])
        self.out.print(bar)
        if failed_steps:
            self.out.print(f"  [bold red]failed steps:[/] {', '.join(failed_steps)}")
        if docs:
            self.out.print(f"  [dim]wrote {docs}[/]")
        self.out.print()


UI = PlainUI()


def md_escape(text):
    return str(text).replace("|", "\\|").replace("\n", " ")


def md_table(header, rows):
    lines = ["| " + " | ".join(header) + " |",
             "| " + " | ".join("---" for _ in header) + " |"]
    lines += ["| " + " | ".join(md_escape(cell) for cell in row) + " |"
              for row in rows]
    return "\n".join(lines)


GENERATED = ("<!-- Generated by tools/report.py. Do not edit by hand; "
             "run the script again. -->")


def summary_md(env, criteria, tests, steps):
    status_mark = {MET: "met", PARTIAL: "**partial**"}
    parts = [GENERATED, "", "# Report summary", "",
             "Success criteria of [docs/project.md](../project.md), measured by "
             "`tools/report.py`. Only `met` is a pass; `not run`, `not measured`, "
             "`stale` and `partial` are not.", "", "## Environment", "",
             md_table(("Key", "Value"), list(env.items())), "",
             "## Criteria", "",
             md_table(("Criterion", "Target", "Measured", "Status", "Source", "Note"),
                      [(c.id, c.target, c.measured,
                        status_mark.get(c.status, f"**{c.status}**"),
                        c.source, c.note) for c in criteria]), "",
             "Details: [accuracy.md](accuracy.md) (synthetic drive, criteria 2 "
             "and 3) and [kitti.md](kitti.md) (KITTI drive, criterion 1 and "
             "information metrics).", "", "## Tests", ""]
    if tests:
        parts.append(md_table(("Test", "Result"), sorted(tests.items())))
    else:
        parts.append("ctest did not run.")
    parts += ["", "## Steps", "",
              md_table(("Step", "Status", "Exit", "Seconds", "Command"),
                       [(s.name, s.status, "" if s.exit_code is None else s.exit_code,
                         f"{s.seconds:.1f}", f"`{shlex.join(s.command)}`"
                         if s.command else s.reason) for s in steps])]
    failed = [s for s in steps if s.status == "failed" and s.reason]
    for step in failed:
        parts += ["", f"### {step.name}: output tail", "", "```text",
                  step.reason, "```"]
    return "\n".join(parts) + "\n"


def accuracy_md(accuracy):
    parts = [GENERATED, "", "# Accuracy on the synthetic drive", ""]
    if accuracy is None:
        return "\n".join(parts + ["`accuracy` did not run.", ""])
    scenario = accuracy.get("scenario", {})
    parts += [f"Program `accuracy`, {accuracy.get('seeds')} seeds, drive "
              f"`{scenario.get('drive')}`, outage "
              f"{scenario.get('outage_start')} s + {scenario.get('outage_duration')} s, "
              f"GNSS every {scenario.get('gnss_period')} s with "
              f"{scenario.get('gnss_noise')} m noise. See "
              "[docs/accuracy.md](../accuracy.md).", "",
              "## Verdicts", "",
              md_table(("Criterion", "Target", "Median", "Min", "Max",
                        "Seeds met", "Verdict"),
                       [(identifier, entry.get("target"), entry.get("median"),
                         entry.get("min"), entry.get("max"),
                         entry.get("seeds_met"), entry.get("verdict"))
                        for identifier, entry in accuracy.get("criteria", {}).items()]),
              "", "## Filter parameters", "",
              md_table(("Parameter", "Value"),
                       list(accuracy.get("ekf", {}).items())),
              "", "## Per seed", ""]

    def fmt(value, digits=3):
        return "n/a" if value is None else f"{value:.{digits}f}"

    rows = []
    for run in accuracy.get("runs", []):
        c2 = run.get("criterion2") or {}
        c3 = run.get("criterion3") or {}
        rows.append((run.get("seed"), fmt(c2.get("raw_gnss_rmse")),
                     fmt(c2.get("filter_rmse")), percent(c2.get("reduction")),
                     fmt(c3.get("t"), 2), fmt(c3.get("filter_error")),
                     fmt(c3.get("imu_only_error")), times(c3.get("ratio"))))
    parts.append(md_table(("Seed", "Raw GNSS RMSE, m", "Filter RMSE, m",
                           "Reduction", "Reacq. t, s", "Filter error, m",
                           "IMU-only error, m", "Ratio"), rows))
    return "\n".join(parts) + "\n"


def kitti_md(metrics, compare, reason):
    parts = [GENERATED, "", "# KITTI drive", "",
             "Information only: criteria 2 and 3 are judged on the synthetic "
             "drive (see [accuracy.md](accuracy.md)). Criterion 1 uses the "
             "filterpy agreement below. See "
             "[docs/kitti-scenario.md](../kitti-scenario.md).", ""]
    if metrics is None:
        return "\n".join(parts + [f"`kitti_run` did not run: {reason}", ""])
    dataset = metrics.get("dataset", {})
    gnss = metrics.get("gnss", {})
    m = metrics.get("metrics", {})
    reacquisition = m.get("reacquisition") or {}

    def fmt(value, digits=3):
        return "n/a" if value is None else f"{value:.{digits}f}"

    parts += [f"Drive `{dataset.get('oxts')}`, {dataset.get('steps')} steps over "
              f"{fmt(dataset.get('duration_s'), 1)} s; {gnss.get('full_fixes')} fixes, "
              f"{gnss.get('outage_fixes')} with the outage "
              f"[{gnss.get('outage_start_s')}, "
              f"{(gnss.get('outage_start_s') or 0) + (gnss.get('outage_length_s') or 0)}) s.",
              "", "## Metrics (from kitti_metrics.json)", "",
              md_table(("Metric", "Value"), [
                  ("raw GNSS RMSE, m", fmt(m.get("raw_gnss_rmse_m"))),
                  ("filter RMSE, m", fmt(m.get("filter_rmse_m"))),
                  ("filter RMSE reduction", percent(m.get("filter_rmse_reduction"))),
                  ("filter with outage, route RMSE, m", fmt(m.get("filter_outage_rmse_route_m"))),
                  ("filter with outage, window RMSE, m", fmt(m.get("filter_outage_rmse_window_m"))),
                  ("IMU only, window RMSE, m", fmt(m.get("imu_only_window_rmse_m"))),
                  ("reacquisition time, s", fmt(reacquisition.get("t_s"), 2)),
                  ("error at reacquisition, filter, m", fmt(reacquisition.get("filter_error_m"))),
                  ("error at reacquisition, IMU only, m", fmt(reacquisition.get("imu_only_error_m"))),
                  ("reacquisition ratio", times(reacquisition.get("ratio"))),
              ]), "", "## Agreement with filterpy (filterpy_compare.json)", ""]
    if compare is None:
        parts.append("The filterpy comparison did not run.")
    else:
        columns = ("east", "north", "yaw", "v", "std_east", "std_north")
        rows = []
        for name, run in compare.get("runs", {}).items():
            diff = run.get("max_abs_diff")
            rows.append((name, run.get("rows"),
                         *(f"{diff[c]:.2g}" if diff else run.get("error", "n/a")
                           for c in columns)))
        parts += [f"filterpy {compare.get('filterpy_version')}; "
                  "max |own - filterpy| per column.", "",
                  md_table(("Run", "Rows", *columns), rows)]
    return "\n".join(parts) + "\n"


# --------------------------------------------------------------------------

def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build-dir", default="build",
                        help="CMake build directory (default build)")
    parser.add_argument("--oxts", default=DEFAULT_OXTS,
                        help=f"KITTI oxts directory (default {DEFAULT_OXTS})")
    parser.add_argument("--seeds", type=int, default=None,
                        help="seeds for accuracy (default: its own default)")
    parser.add_argument("--python", default=sys.executable,
                        help="Python with numpy, matplotlib and filterpy, "
                             "for ref_ekf.py (default: this one)")
    parser.add_argument("--skip-build", action="store_true",
                        help="do not configure, build or run ctest")
    parser.add_argument("--strict", action="store_true",
                        help="exit 1 when any criterion is not met")
    parser.add_argument("--no-docs", action="store_true",
                        help="print the table only; do not write docs/report")
    parser.add_argument("--plain", action="store_true",
                        help="plain text output even when rich is installed")
    return parser.parse_args()


def main():
    global UI
    args = parse_arguments()
    if rich is not None and not args.plain:
        UI = RichUI()
    build_dir = (ROOT / args.build_dir).resolve()
    out = build_dir / "report"
    out.mkdir(parents=True, exist_ok=True)
    steps = []

    # 1. Build and tests.
    tests = {}
    built = True
    if args.skip_build:
        skip_step(steps, "build", "--skip-build")
        skip_step(steps, "ctest", "--skip-build")
    else:
        configure = run_step(steps, "configure", ["cmake", "-S", ROOT, "-B", build_dir])
        build = run_step(steps, "build", ["cmake", "--build", build_dir, "--parallel"]) \
            if configure.ok else skip_step(steps, "build", "configure failed")
        built = build.ok
        junit = out / "ctest.xml"
        remove(junit)
        if built:
            run_step(steps, "ctest", ["ctest", "--test-dir", build_dir,
                                      "--output-on-failure", "--output-junit",
                                      # ctest resolves it against --test-dir.
                                      os.path.relpath(junit, build_dir)])
            tests = read_junit(junit)
        else:
            skip_step(steps, "ctest", "build failed")

    # 2. Synthetic drive: criteria 2 and 3.
    accuracy = None
    accuracy_program = build_dir / "accuracy"
    if accuracy_program.is_file():
        command = [accuracy_program, "--out", out / "accuracy"]
        if args.seeds is not None:
            command += ["--seeds", args.seeds]
        if run_step(steps, "accuracy", command).ok:
            accuracy = read_json(out / "accuracy" / "accuracy.json")
    else:
        skip_step(steps, "accuracy", f"{accuracy_program} is not built")

    # 3. KITTI drive: criterion 1 and the information metrics.
    kitti = out / "kitti"
    oxts = (ROOT / args.oxts).resolve()
    kitti_program = build_dir / "kitti_run"
    compare_path = kitti / "filterpy_compare.json"
    remove(compare_path)
    metrics = compare = None
    kitti_reason = ""
    figure = None
    if not (oxts / "timestamps.txt").is_file():
        kitti_reason = f"no KITTI oxts data at {oxts} (see data/README.md)"
        for name in ("ref_ekf", "kitti_run", "filterpy compare"):
            skip_step(steps, name, kitti_reason)
    elif python_module_version(args.python, "filterpy") is None:
        kitti_reason = (f"{args.python} cannot import filterpy; install "
                        "tools/requirements.txt or pass --python")
        for name in ("ref_ekf", "kitti_run", "filterpy compare"):
            skip_step(steps, name, kitti_reason)
    elif not kitti_program.is_file():
        kitti_reason = f"{kitti_program} is not built"
        for name in ("ref_ekf", "kitti_run", "filterpy compare"):
            skip_step(steps, name, kitti_reason)
    else:
        ref = [args.python, ROOT / "foreign_impl/ref_ekf.py", oxts, "--out", kitti]
        reference = run_step(steps, "ref_ekf", ref)
        if reference.ok:
            figure = kitti / "trajectories.png"
            own = run_step(steps, "kitti_run", [
                kitti_program, "--oxts", oxts,
                "--gnss", kitti / "gnss_full.csv",
                "--gnss-outage", kitti / "gnss_outage.csv", "--out", kitti])
            if own.ok:
                metrics = read_json(kitti / "kitti_metrics.json")
                if run_step(steps, "filterpy compare", ref + [
                        "--own", kitti / "kitti_estimates.csv",
                        "--own-outage", kitti / "kitti_estimates_outage.csv",
                        "--json", compare_path]).ok:
                    compare = read_json(compare_path)
            else:
                kitti_reason = "kitti_run failed"
                skip_step(steps, "filterpy compare", kitti_reason)
        else:
            kitti_reason = "ref_ekf.py failed"
            skip_step(steps, "kitti_run", kitti_reason)
            skip_step(steps, "filterpy compare", kitti_reason)

    # 4. Verdicts and output.
    targets = read_targets()
    criteria = evaluate(targets, accuracy, compare, tests, figure)
    env = environment(build_dir, args.python)
    failed_steps = [s.name for s in steps if s.status == "failed"]

    docs = None
    if not args.no_docs:
        DOCS_REPORT.mkdir(parents=True, exist_ok=True)
        (DOCS_REPORT / "summary.md").write_text(summary_md(env, criteria, tests, steps))
        (DOCS_REPORT / "accuracy.md").write_text(accuracy_md(accuracy))
        (DOCS_REPORT / "kitti.md").write_text(kitti_md(metrics, compare, kitti_reason))
        docs = f"{DOCS_REPORT.relative_to(ROOT)}/summary.md, accuracy.md, kitti.md"
    UI.final(env, criteria, failed_steps, docs)

    if not built:
        return 1
    if args.strict and any(c.status != MET for c in criteria):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
