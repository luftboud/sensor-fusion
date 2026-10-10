#!/usr/bin/env bash
# Read NUL-separated commit messages on stdin. Exit 1 if any subject line
# breaks the Conventional Commits rules used by this repository.
set -euo pipefail

subject_re='^(feat|fix|docs|test|refactor|perf|ci|build|chore)(\([a-z0-9]+(-[a-z0-9]+)*\))?: [a-z0-9].*'

fail=0
while IFS= read -r -d '' message; do
  [ -n "$message" ] || continue
  subject="${message%%$'\n'*}"
  subject="${subject%$'\r'}"

  if [ "${#subject}" -gt 72 ]; then
    echo "subject longer than 72 characters: ${subject}"
    fail=1
    continue
  fi
  if [[ "$subject" == *. ]]; then
    echo "subject must not end with a period: ${subject}"
    fail=1
    continue
  fi
  if [[ ! "$subject" =~ $subject_re ]]; then
    echo "subject does not match Conventional Commits: ${subject}"
    fail=1
    continue
  fi

  description="${subject#*: }"
  if [[ "$description" =~ ^wip($|[[:space:]]) ]]; then
    echo "subject must not start with wip: ${subject}"
    fail=1
    continue
  fi

  rest="${message#"$subject"}"
  if [ -n "$rest" ]; then
    newline=$'\n'
    body="${rest#"$newline"}"
    if [ -n "$body" ] && [ "${body:0:1}" != "$newline" ]; then
      echo "blank line required between subject and body: ${subject}"
      fail=1
    fi
  fi
done

exit "$fail"
