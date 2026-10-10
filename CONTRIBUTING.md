# Contributing

`main` is the only long-lived branch. Land changes through a pull request, and merge that pull request with **Rebase and merge**. Each commit on the branch is replayed onto `main`. There is no merge commit and no squash, so every commit you push is part of the permanent history. Clean it up before review.

A pull request needs one approving review. GitHub does not let the author approve their own pull request, so a second account with write access has to approve it.

## Branches

Create the branch from an up-to-date `main`:

```text
<github-login>/<type>/<kebab-summary>
```

`type` is one of `feat`, `fix`, `docs`, `test`, `refactor`, `perf`, `ci`, `build`, `chore`. The summary is lowercase kebab-case.

```text
luftboud/feat/nmea-parser
```

Pull request checks reject any other name. `main` itself is not checked.

## Commits

Use [Conventional Commits](https://www.conventionalcommits.org/). A scope is optional:

```text
type: subject
type(scope): subject
```

- The whole first line is at most 72 characters.
- The subject is English, imperative, and starts with a lowercase letter or a digit: `add nmea parser`, not `Added nmea parser`.
- Do not end the subject with a period.
- Do not start the subject with `wip`. Finish the commit or drop it before the pull request.
- A scope, when present, is kebab-case: `feat(io): parse gga sentences`.
- If you add a body, leave a blank line after the subject. Wrap the body at 72 characters. The blank line is checked; the wrap is not.

Allowed types: `feat`, `fix`, `docs`, `test`, `refactor`, `perf`, `ci`, `build`, `chore`.

```text
feat: add nmea parser

Reject sentences whose checksum does not match.
```

Because rebase merge keeps every commit, do not leave fixup commits on the branch. Squash or reword them locally before asking for review:

```bash
git rebase -i origin/main
```

## Updating a pull request

Rebase onto the latest `main`. Do not merge `main` into the feature branch.

```bash
git fetch origin
git rebase origin/main
git push --force-with-lease
```

`--force-with-lease` is only for your feature branch. Never force-push `main`.

If GitHub offers **Update branch**, skip it when that button merges `main` into your branch. Rebase locally instead.

## What CI runs

On every push, and again on the pull request:

| Check | What it does |
| --- | --- |
| `CI / build (gcc)` | Configure and build with GCC, `-Wall -Wextra -Wpedantic -Werror` |
| `CI / build (clang)` | Same with Clang |
| `CI / sanitizers` | Clang Debug build with AddressSanitizer and UndefinedBehaviorSanitizer |
| `Lint / clang-tidy` | `clang-tidy` once a `.cpp` file exists; skipped until then |
| `Lint / commit-messages` | Subject lines of the new commits |
| `Lint / branch-name` | Pull request head branch only |
| `Lint / actionlint` | The workflow files themselves |

There is no clang-format check. Sanitizer flags live in the workflow, not in `CMakeLists.txt`.

`clang-tidy` analyzes translation units and the `include/sensor_fusion/` headers they include. Header-only code is invisible to it until some `.cpp` includes that header. The enabled groups are `bugprone`, `clang-analyzer`, `performance`, `modernize`, and `readability`, with trailing return types, short identifier names, and magic numbers left off. Findings are errors.

The library is header-only today, so the build and sanitizer jobs configure successfully and compile nothing. The same flags apply to the first real target.

## Run the checks locally

```bash
cmake -S . -B build-gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Werror"
cmake --build build-gcc

cmake -S . -B build-clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Werror"
cmake --build build-clang

cmake -S . -B build-san -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-omit-frame-pointer"
cmake --build build-san

git fetch origin main
git log --reverse --format='%B%x00' origin/main..HEAD | .github/scripts/check-commits.sh
```

When a `.cpp` file exists:

```bash
cmake -S . -B build-tidy -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_COMPILER=clang++
clang-tidy -p build-tidy path/to/file.cpp --header-filter='include/sensor_fusion/'
```

## Repository settings

These toggles have to stay in place on GitHub so the rules above are what the UI offers:

- Allow rebase merging: on. Allow merge commits: off. Allow squash merging: off.
- Automatically delete head branches: on.
- Branch protection on `main`: pull request required, one approving review, stale reviews dismissed, linear history required, the branch must be up to date, and administrators are not exempt.
- Required status checks: `CI / build (gcc)`, `CI / build (clang)`, `CI / sanitizers`, `Lint / clang-tidy`, `Lint / commit-messages`, `Lint / branch-name`, `Lint / actionlint`.
