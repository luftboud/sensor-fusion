# Runs apps/report/kitti_run on a small drive that this script writes and
# checks its outputs, its exit status and its error handling.
#
#   cmake -DKITTI_RUN=<program> -DWORK=<dir> -P kitti_run_tests.cmake

foreach(variable KITTI_RUN WORK)
  if(NOT DEFINED ${variable})
    message(FATAL_ERROR "${variable} is not set")
  endif()
endforeach()

file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")

# The input drive, written into the work directory so that nothing generated
# is kept in the repository. The five OXTS records are made up, not taken
# from KITTI: the car goes straight at 10 m/s on a heading of 0.5 rad and
# turns slowly, with samples 0.1 s apart.
set(FIXTURE "${WORK}/fixture")
file(WRITE "${FIXTURE}/oxts/timestamps.txt"
  "2011-09-26 13:02:25.050000000\n"
  "2011-09-26 13:02:25.150000000\n"
  "2011-09-26 13:02:25.250000000\n"
  "2011-09-26 13:02:25.350000000\n"
  "2011-09-26 13:02:25.450000000\n")
set(latitudes 49.0 49.00001 49.00002 49.00003 49.00004)
set(longitudes 8.4 8.40001 8.40002 8.400030000000001 8.40004)
set(yaws 0.5 0.501 0.502 0.503 0.504)
foreach(index RANGE 4)
  list(GET latitudes ${index} lat)
  list(GET longitudes ${index} lon)
  list(GET yaws ${index} yaw)
  file(WRITE "${FIXTURE}/oxts/data/000000000${index}.txt"
    "${lat} ${lon} 110.0 0.01 0.02 ${yaw} 5.0 8.7 10.0 0.0 0.0 0.1 0.0 9.8 "
    "0.1 0.0 9.8 0.0 0.0 0.01 0.0 0.0 0.01 0.5 0.1 4 10 5 5 6\n")
endforeach()
# gnss.csv holds three fixes between the samples, at 0.05 s, 0.15 s and
# 0.35 s. gnss_outage.csv cuts the outage window [0.125, 0.25) s, so the fix
# at 0.15 s is gone. gnss_none.csv holds only the header: a drive without
# GNSS, whose filter run is the IMU-only run.
file(WRITE "${FIXTURE}/gnss.csv"
  "t,east,north\n5e-02,0.5,0.25\n1.5e-01,1.0,0.5\n3.5e-01,2.5,1.5\n")
file(WRITE "${FIXTURE}/gnss_outage.csv"
  "t,east,north\n5e-02,0.5,0.25\n3.5e-01,2.5,1.5\n")
file(WRITE "${FIXTURE}/gnss_none.csv" "t,east,north\n")
# Not cut from gnss.csv: the last fix is 0.1 m further north.
file(WRITE "${FIXTURE}/gnss_other.csv"
  "t,east,north\n5e-02,0.5,0.25\n3.5e-01,2.5,1.6\n")

# Reports a failed check and keeps going, so that one run shows them all.
function(fail text)
  message(SEND_ERROR "${text}")
  set_property(GLOBAL APPEND PROPERTY kitti_run_failures "${text}")
endfunction()

# Runs kitti_run with ARGN. Sets run_exit, run_stdout and run_stderr.
function(run_kitti_run)
  execute_process(
    COMMAND "${KITTI_RUN}" ${ARGN}
    RESULT_VARIABLE exit_code
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
  set(run_exit "${exit_code}" PARENT_SCOPE)
  set(run_stdout "${out}" PARENT_SCOPE)
  set(run_stderr "${err}" PARENT_SCOPE)
endfunction()

# Expects exit status EXPECTED from kitti_run with ARGN.
function(expect_exit expected)
  run_kitti_run(${ARGN})
  if(NOT run_exit STREQUAL "${expected}")
    fail("kitti_run ${ARGN}: exit ${run_exit}, expected ${expected}\n${run_stderr}")
  endif()
  if(NOT run_stdout STREQUAL "")
    fail("kitti_run ${ARGN}: wrote to stdout: ${run_stdout}")
  endif()
endfunction()

set(inputs
  --oxts "${FIXTURE}/oxts"
  --gnss "${FIXTURE}/gnss.csv"
  --gnss-outage "${FIXTURE}/gnss_outage.csv"
  --outage-start 0.125
  --outage-length 0.125)
set(full_name kitti_estimates.csv)
set(outage_name kitti_estimates_outage.csv)
set(metrics_name kitti_metrics.json)

# Two runs into fresh directories, one of them nested, give the same files.
expect_exit(0 ${inputs} --out "${WORK}/first")
expect_exit(0 ${inputs} --out "${WORK}/second/nested")
foreach(name ${full_name} ${outage_name} ${metrics_name})
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E compare_files
            "${WORK}/first/${name}" "${WORK}/second/nested/${name}"
    RESULT_VARIABLE different)
  if(different)
    fail("${name} differs between two runs")
  endif()
endforeach()
file(GLOB leftovers "${WORK}/first/*.tmp" "${WORK}/second/nested/*.tmp")
if(leftovers)
  fail("temporary files left behind: ${leftovers}")
endif()

# One row per IMU step after the header. The first row is the initial
# state: east 0, north 0, the OXTS yaw and vf, and the initial std.
foreach(name ${full_name} ${outage_name})
  file(STRINGS "${WORK}/first/${name}" rows)
  list(LENGTH rows row_count)
  if(NOT row_count EQUAL 6)
    fail("${name}: ${row_count} lines, expected the header and 5 rows")
    continue()
  endif()
  list(GET rows 0 header)
  list(GET rows 1 first_row)
  if(NOT header STREQUAL "t,east,north,yaw,v,std_east,std_north,updated")
    fail("${name}: header is '${header}'")
  endif()
  if(NOT first_row STREQUAL "0,0,0,0.5,10,2,2,0")
    fail("${name}: first row is '${first_row}'")
  endif()
  # CMake regular expressions have no {7}, so the field is repeated.
  string(REPEAT "[-0-9.e]+," 7 number_fields)
  foreach(row ${rows})
    if(NOT row STREQUAL header AND NOT row MATCHES
       "^${number_fields}[01]$")
      fail("${name}: row '${row}' is not seven numbers and a flag")
    endif()
  endforeach()
endforeach()

# The full run uses its three fixes, the outage run the two outside the
# window.
file(STRINGS "${WORK}/first/${full_name}" updated_rows REGEX ",1$")
list(LENGTH updated_rows updated_count)
if(NOT updated_count EQUAL 3)
  fail("${full_name}: ${updated_count} updated rows, expected 3")
endif()
file(STRINGS "${WORK}/first/${outage_name}" updated_rows REGEX ",1$")
list(LENGTH updated_rows updated_count)
if(NOT updated_count EQUAL 2)
  fail("${outage_name}: ${updated_count} updated rows, expected 2")
endif()

# The metrics file. Values that depend on the filter are checked for presence
# and sign; the rest is exact.
file(READ "${WORK}/first/${metrics_name}" metrics)
macro(expect_json expected)
  string(JSON actual ERROR_VARIABLE json_error GET "${metrics}" ${ARGN})
  if(json_error)
    fail("${metrics_name}: ${json_error}")
  elseif(NOT actual STREQUAL "${expected}")
    fail("${metrics_name}: ${ARGN} is '${actual}', expected '${expected}'")
  endif()
endmacro()
# string(JSON GET) prints numbers with 17 digits, so a decimal value is
# checked as it stands in the file instead.
macro(expect_text line)
  string(FIND "${metrics}" "${line}\n" position)
  if(position EQUAL -1)
    fail("${metrics_name}: no line '${line}'")
  endif()
endmacro()
expect_json(1 schema)
expect_json(kitti_run producer)
expect_json("${FIXTURE}/oxts" dataset oxts)
expect_json(5 dataset steps)
expect_text("    \"duration_s\": 0.4")
expect_json(3 gnss full_fixes)
expect_json(2 gnss outage_fixes)
expect_text("    \"outage_start_s\": 0.125,")
expect_text("    \"outage_length_s\": 0.125")
# The EkfConfig defaults, under the names of the flags.
expect_text("    \"initial_position_std\": 2,")
expect_text("    \"initial_yaw_std\": 0.2,")
expect_text("    \"initial_speed_std\": 1,")
expect_text("    \"accel_noise_density\": 0.3,")
expect_text("    \"gyro_noise_density\": 0.03,")
expect_text("    \"gnss_position_std\": 2")
# The first fix after the window is at 0.35 s, consumed on the step at
# 0.4 s, so the step before it is at 0.3 s: 300000000 ns * 1e-9.
expect_json(0.30000000000000004 metrics reacquisition t_s)
foreach(key raw_gnss_rmse_m filter_rmse_m filter_outage_rmse_route_m
            filter_outage_rmse_window_m imu_only_window_rmse_m)
  string(JSON value ERROR_VARIABLE json_error GET "${metrics}" metrics ${key})
  if(json_error OR NOT value MATCHES "^[0-9][0-9.e+-]*$")
    fail("${metrics_name}: ${key} is '${value}', expected a positive number")
  endif()
endforeach()
# Below zero here: over five steps the filter is still worse than raw GNSS.
string(JSON value ERROR_VARIABLE json_error GET "${metrics}"
       metrics filter_rmse_reduction)
if(json_error OR NOT value MATCHES "^-?[0-9][0-9.e+-]*$")
  fail("${metrics_name}: filter_rmse_reduction is '${value}'")
endif()
foreach(key filter_error_m imu_only_error_m ratio)
  string(JSON value ERROR_VARIABLE json_error GET "${metrics}"
         metrics reacquisition ${key})
  if(json_error OR NOT value MATCHES "^[0-9][0-9.e+-]*$")
    fail("${metrics_name}: reacquisition.${key} is '${value}'")
  endif()
endforeach()
# Exactly these sections, in this order. string(JSON MEMBER) sorts the keys,
# so the order is read from the text: top-level keys are indented by two.
string(REGEX MATCHALL "\n  \"[a-z_]+\":" sections "${metrics}")
string(REGEX REPLACE "[\n \":]" "" sections "${sections}")
if(NOT sections STREQUAL "schema;producer;dataset;gnss;config;metrics")
  fail("${metrics_name}: top-level keys are '${sections}'")
endif()

# Cross-checks between runs, since the exact values depend on the filter.
# A run without GNSS is the IMU-only run, so its outage-window RMSE is the
# IMU-only window RMSE of the first run.
expect_exit(0 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss_none.csv"
            --gnss-outage "${FIXTURE}/gnss_none.csv" --out "${WORK}/no_gnss"
            --outage-start 0.125 --outage-length 0.125)
file(READ "${WORK}/no_gnss/${metrics_name}" no_gnss)
string(JSON imu_only_rmse GET "${metrics}" metrics imu_only_window_rmse_m)
string(JSON no_gnss_rmse GET "${no_gnss}" metrics filter_outage_rmse_window_m)
if(NOT imu_only_rmse STREQUAL no_gnss_rmse)
  fail("IMU-only window RMSE ${imu_only_rmse} is not that of a run without "
       "GNSS, ${no_gnss_rmse}")
endif()
string(JSON no_gnss_raw TYPE "${no_gnss}" metrics raw_gnss_rmse_m)
if(NOT no_gnss_raw STREQUAL "NULL")
  fail("raw_gnss_rmse without fixes is ${no_gnss_raw}, expected null")
endif()
# The raw GNSS error uses every fix, not the ones left after the outage. A
# window with no fix inside cuts nothing, so the full file is the outage file.
expect_exit(0 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${FIXTURE}/gnss.csv" --out "${WORK}/no_outage"
            --outage-start 0.2 --outage-length 0.1)
file(READ "${WORK}/no_outage/${metrics_name}" no_outage)
string(JSON raw GET "${metrics}" metrics raw_gnss_rmse_m)
string(JSON no_outage_raw GET "${no_outage}" metrics raw_gnss_rmse_m)
if(NOT raw STREQUAL no_outage_raw)
  fail("raw_gnss_rmse ${raw} changes with the outage file (${no_outage_raw})")
endif()
# Without an outage the outage run is the full run.
string(JSON filter GET "${no_outage}" metrics filter_rmse_m)
string(JSON outage GET "${no_outage}" metrics filter_outage_rmse_route_m)
if(NOT filter STREQUAL outage)
  fail("without an outage, outage_rmse ${outage} is not filter_rmse ${filter}")
endif()

# A window with no fix after it has no reacquisition, and one with no step
# inside it has no window RMSE. Both are null, not zero. No fix lies in the
# window, so the outage file is the full one.
expect_exit(0 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${FIXTURE}/gnss.csv" --out "${WORK}/late"
            --outage-start 0.41 --outage-length 0.05)
file(READ "${WORK}/late/${metrics_name}" late)
foreach(key reacquisition filter_outage_rmse_window_m imu_only_window_rmse_m)
  string(JSON type ERROR_VARIABLE json_error TYPE "${late}" metrics ${key})
  if(NOT type STREQUAL "NULL")
    fail("${metrics_name}: ${key} is ${type} for a window after the last fix")
  endif()
endforeach()

# A filter flag changes the result.
expect_exit(0 ${inputs} --out "${WORK}/tuned" --gnss-position-std 0.5)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E compare_files
          "${WORK}/first/${full_name}" "${WORK}/tuned/${full_name}"
  RESULT_VARIABLE different)
if(NOT different)
  fail("--gnss-position-std did not change the estimates")
endif()
# The metrics record the flag value that was used.
file(READ "${WORK}/tuned/${metrics_name}" tuned)
string(JSON tuned_std GET "${tuned}" config gnss_position_std)
if(NOT tuned_std STREQUAL "0.5")
  fail("config.gnss_position_std is '${tuned_std}', expected 0.5")
endif()

# The oxts path is escaped, so a quote in it keeps the file valid JSON.
set(quoted_oxts "${WORK}/say \"hi\"/oxts")
file(COPY "${FIXTURE}/oxts/" DESTINATION "${quoted_oxts}")
expect_exit(0 --oxts "${quoted_oxts}" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${FIXTURE}/gnss_outage.csv" --out "${WORK}/quoted"
            --outage-start 0.125 --outage-length 0.125)
file(READ "${WORK}/quoted/${metrics_name}" quoted)
string(JSON quoted_path ERROR_VARIABLE json_error GET "${quoted}" dataset oxts)
if(json_error OR NOT quoted_path STREQUAL quoted_oxts)
  fail("dataset.oxts is '${quoted_path}', expected '${quoted_oxts}' "
       "${json_error}")
endif()

# Help goes to stdout.
run_kitti_run(--help)
if(NOT run_exit STREQUAL "0" OR NOT run_stdout MATCHES "^usage: kitti_run")
  fail("--help: exit ${run_exit}, stdout '${run_stdout}'")
endif()

# Bad command lines exit with 2.
expect_exit(2)
expect_exit(2 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss.csv")
expect_exit(2 ${inputs} --unknown 1)
expect_exit(2 ${inputs} --out)
expect_exit(2 ${inputs} --out "")
expect_exit(2 ${inputs} --accel-noise-density abc)
expect_exit(2 ${inputs} --accel-noise-density nan)
expect_exit(2 ${inputs} --gnss-position-std 0)
expect_exit(2 ${inputs} --initial-yaw-std -1)
expect_exit(2 ${inputs} --outage-length -1)
expect_exit(2 ${inputs} --outage-start inf)

# Bad input data exits with 1 and leaves the earlier outputs alone.
file(READ "${WORK}/first/${full_name}" before)
expect_exit(1 --oxts "${WORK}/missing" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${FIXTURE}/gnss_outage.csv" --out "${WORK}/first")
expect_exit(1 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/oxts/timestamps.txt"
            --gnss-outage "${FIXTURE}/gnss_outage.csv" --out "${WORK}/first")
expect_exit(1 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${WORK}/missing.csv" --out "${WORK}/first")
# An outage file that is not the full file without the window: the full file
# itself, which keeps the fix at 0.15 s inside the window; a file cut with
# another window, which lacks the fix at 0.15 s outside [0.3, 0.4); and a
# file whose fix differs from the full one.
set(window --outage-start 0.125 --outage-length 0.125)
expect_exit(1 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${FIXTURE}/gnss.csv" --out "${WORK}/first" ${window})
expect_exit(1 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${FIXTURE}/gnss_outage.csv" --out "${WORK}/first"
            --outage-start 0.3 --outage-length 0.1)
expect_exit(1 --oxts "${FIXTURE}/oxts" --gnss "${FIXTURE}/gnss.csv"
            --gnss-outage "${FIXTURE}/gnss_other.csv" --out "${WORK}/first"
            ${window})
file(READ "${WORK}/first/${full_name}" after)
if(NOT before STREQUAL after)
  fail("a failed run changed ${full_name}")
endif()

# An output path under a regular file cannot be created.
expect_exit(1 ${inputs} --out "${WORK}/first/${full_name}/out")

get_property(failures GLOBAL PROPERTY kitti_run_failures)
list(LENGTH failures failure_count)
if(failure_count GREATER 0)
  message(FATAL_ERROR "${failure_count} kitti_run checks failed")
endif()
message(STATUS "All kitti_run tests passed")
