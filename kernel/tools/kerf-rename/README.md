# kerf-rename

Renames Kerf's code to .NET conventions from a reviewed name map (phase10-plan.md 2.3, docs/design-rules.md section 1).
Unit 10E ran it assembly by assembly over Kerf.slnx and the benchmarks (docs/api/kerf.map, with the renames it dropped in
docs/api/kerf-keeps.map); Bento is updated afterwards with the same maps (10N).

It is a Roslyn tool: it loads a solution with MSBuildWorkspace, binds every name that could refer to something renamed,
and rewrites the text. It never guesses: a rename that would collide is dropped and reported.

```sh
dotnet build kernel/tools/kerf-rename -c Release
TOOL=kernel/tools/kerf-rename/bin/Release/net10.0/kerf-rename.dll
PROPS='-p:OcctSourceDir=…\occt-src\ -p:OcctTestDataDir=…\opencascade-dataset-7.9.0\'

# Draft the systematic part of a map, for review
dotnet $TOOL draft --solution Kerf.slnx --assembly Kerf.Kernel.Foundation --package gp --package math \
    --seed seed.map --out draft.map $PROPS

# Apply a reviewed map (--dry-run to see the report only)
dotnet $TOOL apply --solution Kerf.slnx --assembly Kerf.Kernel.Foundation [--package gp …] --map reviewed.map \
    --report report.txt --applied applied.map --rename-files $PROPS
```

- `--solution` (or one or more `--project`) is the code whose **references** are updated: every project in it.
- `--assembly` (and optionally `--package`, OCCT's package prefixes) is the **scope**: only symbols declared there are
  renamed. One assembly or one package at a time.
- `-p:Name=Value` passes MSBuild properties to the load (Configuration is Release).
- **Another repository** (Bento): `--solution` is that repository's solution and `--assembly` names Kerf's assemblies,
  which it references as packages or projects; the map is the *applied* map of the kernel's run (below).

## The map

A text file, one entry per line, sorted by kind and id, so a review is a diff. `#` starts a comment.

```text
rule parameters on                  # theX -> x, and OCCT's older PascalCase parameters -> camelCase
rule fields on                      # non-public instance fields: myX -> _x, coord -> _coord
rule statics on                     # non-public static fields: s_x if mutable, X if constant or read-only
rule locals on                      # aX, anX -> x, and PascalCase locals -> camelCase
rule methods on                     # camelCase methods (file-static helpers) -> PascalCase

package gp Kerf.Geometry            # where the drafter puts a package's types

alias math_Vector RealVector        # a using alias, in C# and in MSBuild's <Using> items

type T:Occt.gp_Pnt Kerf.Geometry.Point3D
member F:Occt.gp_TrsfForm.gp_Identity Identity
member M:Occt.gp_Pnt.Distance(Occt.gp_Pnt@)~System.Double DistanceTo
property M:Occt.math_FunctionRoot.NbIterations~System.Int32 IterationCount
param M:Occt.gp_Mat.SetCross(Occt.gp_XYZ@)#theRef other
keep F:Occt.gp.s_Origin             # no rule applies to this symbol
```

- **Ids** are Roslyn's documentation comment ids of the original definitions: the same for source and for metadata, so
  the map applies to Bento's references to Kerf's packages. A parameter is its method's id, `#`, its declared name.
- **type** gives the new full name; the namespace part moves the type. A file's namespace declaration moves when all
  its types move to the same place; otherwise the type keeps its namespace and the report says why.
- **property** turns a parameterless method into a get-only property: declarations (expression, block and abstract
  bodies), overrides in any assembly, calls (`x.IsDone()` → `x.IsDone`; a call made as a statement becomes `_ = …`) and
  crefs.
- **keep** exempts a symbol from the rules. The applied map adds one, with its reason, for every rename the run dropped.

`draft` writes the systematic part for review: each type of a package under the package's namespace, without its
prefix (`--seed` gives the `package` lines and any names already reviewed); enum members without the prefix; and the
parameterless `NbXxx` (→ `XxxCount`), `IsXxx` and `HasXxx` methods that are cheap getters as properties. Methods that
compute, or can't become properties, are listed commented out with the reason.

## What it updates

Every reference in the solution, found by binding, not by text:

- declarations and uses, including overrides (renamed with their root), named arguments, `nameof`, attributes,
  `using static` and `using` aliases, qualified names (`Occt.gp_Pnt`, `global::Occt.gp_Pnt`, an alias `O.gp_Pnt`);
- XML documentation: crefs and `<param>`/`<paramref>` names;
- `[UnsafeAccessor(Name = "…")]` strings naming a renamed member (the modelling layer's 15);
- MSBuild `<Using Include="…" Alias="…">` items (global usings);
- `using` directives: added where a moved type is now needed, or the reference qualified in full where a new `using`
  would make another name ambiguous, or where the new name would mean something else at that point (a member called
  `Scale` hiding the enum member `TransformForm.Scale`);
- committed generated files (the forwarding layer's `*.g.cs`), which it reports, since their generator still writes the
  old names;
- with `--rename-files`, a file named after a renamed type.

## Collisions: reported, not guessed

A rename is dropped, and listed in the report and the applied map, when:

- a parameter or local would take a name another symbol in the same body already has (`aP` and `theP` both becoming
  `p`), or a reserved word (`theRef`, `anObject`); locals give way before parameters;
- a member would share its name with another member of its type, a base or a derived type, or with its type; or a
  method would become a property beside an overload of the same name. Two methods of different names never become
  overloads of one name (unit 10E: `length(…, double?)` beside `Length(…, double)` changed which one a call bound to);
  10E gave such methods a `Core` suffix in the map;
- a method would become a property but is used as a delegate, has attributes, implements an interface, or overrides a
  method the map doesn't convert;
- two types would get one full name.

## What it doesn't do

- **Strings.** A string naming a renamed type (`ToString()`'s `"gp_Pnt(…)"`, exception messages, `"Occt.NCollection_Map`1"`
  in an analyzer) is reported, never changed: messages, logs and reflection need judgement.
- **Reflection and runtime type names:** `GetType().Name`, `UnsafeAccessorType("Occt.BRep_Builder, …")` strings (reported),
  Bento's reflection over property names.
- **Generators:** the forwarding layer's generator and stepgen still write OCCT's names; their output is renamed.
- **Code outside the solution:** the oracle tests and benchmarks (kernel/OcctNet.slnx) need their own run, with
  `--project`.
- **Extension methods of a type that moves** aren't followed to their callers' `using`s (none in gp and math).
- **Comments** other than XML documentation's crefs and parameter names.

## Speed

Measured on the owner's machine (shared with other helpers), Kerf.slnx (16 projects, about 5,800 files):

| Run | Planned renames | Edits | Files | Time |
|---|---:|---:|---:|---:|
| gp and math (the pilot), applied | 5,500 | 102,513 | 2,421 | 33 s |
| All of OcctNet.Foundation (123k lines), a drafted map, `--dry-run` | 20,100 | 257,774 | 4,716 | 45 s |
| All of OcctNet.ModelingAlgorithms (576k lines), a drafted map, `--dry-run` | 83,900 | 456,113 | 1,991 | 47 s |

About 25 s of each run is loading and compiling the solution; the scan takes 13 to 25 s. **The whole kernel**, four
runs, takes about four minutes of the tool's time; each step's build (a minute) and tests (about ten minutes without the
corpus) cost far more.

It is one pass: load and compile the solution once, scan every document in parallel, binding only tokens whose text
is a name the plan renames, then write. Roslyn's `Renamer.RenameSymbolAsync` was measured too: once warm, it
renames one symbol in 0.1 to 3 s (`gp_Pnt`, 1,458 documents, took 32 s cold), but each call returns a new solution
whose changed projects must be recompiled before the next, and a run renames tens of thousands of symbols (84,000 for
ModelingAlgorithms). It also can't turn methods into properties, move types between namespaces, add `using`s or apply
the collision rules, which are most of the work.
