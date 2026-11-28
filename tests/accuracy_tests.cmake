# Runs apps/report/accuracy with two seeds and checks its output, that two
# runs give the same file, and its argument checks. The criteria values are
# not asserted here: they are the result the report shows.
#
#   cmake -DACCURACY=<program> -DWORK=<dir> -P accuracy_tests.cmake

foreach(variable ACCURACY WORK)
  if(NOT DEFINED ${variable})
    message(FATAL_ERROR "${variable} is not set")
  endif()
endforeach()

file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")

function(fail text)
  message(FATAL_ERROR "accuracy_tests: ${text}")
endfunction()

# Runs accuracy with the given arguments and sets run_exit, run_stdout and
# run_stderr in the caller's scope.
function(run_accuracy)
  execute_process(
    COMMAND "${ACCURACY}" ${ARGN}
    RESULT_VARIABLE exit_code
    OUTPUT_VARIABLE stdout_text
    ERROR_VARIABLE stderr_text)
  set(run_exit "${exit_code}" PARENT_SCOPE)
  set(run_stdout "${stdout_text}" PARENT_SCOPE)
  set(run_stderr "${stderr_text}" PARENT_SCOPE)
endfunction()

# Two seeds, the first run.
run_accuracy(--seeds 2 --out "${WORK}/first")
if(NOT run_exit EQUAL 0)
  fail("accuracy --seeds 2: exit ${run_exit}\n${run_stderr}")
endif()
if(NOT run_stdout STREQUAL "")
  fail("accuracy wrote to stdout: ${run_stdout}")
endif()
foreach(criterion 2 3)
  if(NOT run_stderr MATCHES "criterion ${criterion} \\(")
    fail("no verdict line for criterion ${criterion}:\n${run_stderr}")
  endif()
endforeach()

set(metrics_name accuracy.json)
set(first_path "${WORK}/first/${metrics_name}")
if(NOT EXISTS "${first_path}")
  fail("${first_path} was not written")
endif()
file(READ "${first_path}" json)

string(JSON schema GET "${json}" schema)
if(NOT schema EQUAL 1)
  fail("schema is ${schema}, expected 1")
endif()
string(JSON producer GET "${json}" producer)
if(NOT producer STREQUAL "accuracy")
  fail("producer is '${producer}', expected 'accuracy'")
endif()
string(JSON seeds GET "${json}" seeds)
if(NOT seeds EQUAL 2)
  fail("seeds is ${seeds}, expected 2")
endif()
string(JSON run_count LENGTH "${json}" runs)
if(NOT run_count EQUAL 2)
  fail("${run_count} runs, expected 2")
endif()

# The first seed has every value, so both criteria are measured.
string(JSON reduction GET "${json}" runs 0 criterion2 reduction)
if(NOT reduction MATCHES "^-?[0-9.]+(e[-+]?[0-9]+)?$")
  fail("criterion 2 reduction of seed 1 is '${reduction}', expected a number")
endif()
string(JSON ratio GET "${json}" runs 0 criterion3 ratio)
if(NOT ratio MATCHES "^[0-9.]+(e[-+]?[0-9]+)?$")
  fail("criterion 3 ratio of seed 1 is '${ratio}', expected a number")
endif()

foreach(criterion 2 3)
  string(JSON verdict GET "${json}" criteria ${criterion} verdict)
  if(NOT verdict MATCHES "^(met|not met)$")
    fail("criterion ${criterion} verdict is '${verdict}'")
  endif()
endforeach()

# The same arguments give the same file.
run_accuracy(--seeds 2 --out "${WORK}/second")
if(NOT run_exit EQUAL 0)
  fail("second run: exit ${run_exit}\n${run_stderr}")
endif()
file(READ "${WORK}/second/${metrics_name}" again)
if(NOT again STREQUAL json)
  fail("accuracy.json differs between two runs")
endif()

# No temporary files are left behind.
file(GLOB leftovers "${WORK}/first/*.tmp")
if(leftovers)
  fail("temporary files left behind: ${leftovers}")
endif()

# Bad arguments exit with 2 and write nothing to stdout.
foreach(bad "0" "1001" "x" "-1" "2.5")
  run_accuracy(--seeds "${bad}" --out "${WORK}/bad")
  if(NOT run_exit EQUAL 2)
    fail("--seeds ${bad}: exit ${run_exit}, expected 2")
  endif()
  if(NOT run_stdout STREQUAL "")
    fail("--seeds ${bad} wrote to stdout")
  endif()
endforeach()
run_accuracy(--seeds)
if(NOT run_exit EQUAL 2)
  fail("--seeds without a value: exit ${run_exit}, expected 2")
endif()
run_accuracy(--out "")
if(NOT run_exit EQUAL 2)
  fail("empty --out: exit ${run_exit}, expected 2")
endif()
run_accuracy(--frobnicate 1)
if(NOT run_exit EQUAL 2)
  fail("unknown flag: exit ${run_exit}, expected 2")
endif()

# Help goes to stdout and exits with 0.
run_accuracy(--help)
if(NOT run_exit EQUAL 0)
  fail("--help: exit ${run_exit}, expected 0")
endif()
if(NOT run_stdout MATCHES "usage: accuracy")
  fail("--help does not print the usage")
endif()
