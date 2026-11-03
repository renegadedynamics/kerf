# Porting a package

The steps for porting one OCCT package, for a person or an agent. Read
[translation-rules.md](translation-rules.md) first.

1. **Check the dependencies.** List the package's `#include`s outside itself. Each must already be ported,
   or be part of this piece of work. Port packages bottom-up; `docs/port-status.md` shows what exists.
2. **Read the whole package before writing.** Note mutators and their twins, friend access, non-zero
   default constructors, unscoped enums and templates.
3. **Port class by class**, one C# file per class in `src/<Assembly>/<package>/`, with the provenance header.
   Port bodies line for line. Don't fix OCCT; mark its bugs (`// OCCT ...`) and list them in
   [occt-issues.md](occt-issues.md).
4. **Build.** Warnings are errors. When OCCTNET0001 fires, restructure as the rules say; never suppress it.
5. **Port the GTests** for the package to `tests/Kerf.Kernel.Foundation.Tests/<package>/` (or the matching test
   project), keeping test names. Run them.
6. **Add oracle tests** for the package's non-trivial computations (`tests/Kerf.Kernel.Oracle.Tests/<package>/`). Wrap
   new native types in conversions in `Oracle.cs`. If build123d.NET's whitelist (`bindings/occt.txt` there) lacks
   a class you need, note it; adding it is a change to that repository.
7. **Add Draw-lite commands** that the package makes possible, if the corpus scan lists them.
8. **Update the status**: `dotnet run tools/port-status.cs`, and a line in `CHANGELOG.md`.

## Working in parallel

Packages in the same dependency layer can be ported at once. Give each worker its own copy of the
repository (or a git worktree), so one worker's half-finished build doesn't break another's, and merge the
new files back. Keep edits to shared files (the gp core, `Standard.cs`) small and reported.

## Done means

- The package builds clean and every ported GTest passes.
- The oracle tests pass, and the bit-identical share is reported.
- No member is left out silently: each omission has a comment, and anything that changes behaviour is in
  `occt-issues.md`.
