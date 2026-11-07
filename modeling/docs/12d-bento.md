# Unit 12D: Bento loads Kerf from its files

The owner's decision (kernel decision 1892): Bento loads Kerf from its files, in a context that isn't collectible and is shared by whatever extension uses it, so it gets ReadyToRun. It's done on Bento's branch `kerf/load-from-files`, from `kerf/0.3.0`, for the owner to merge. Kerf itself didn't change; the rig has two new rows (decisions 22201 and 22202).

## What it gives

Measured through Bento's real `ExtensionLoader` and the extension's own functions, in a small harness, each in a new process, on 0.3.0's packages. A loaded machine, five pairs back to back, so only the comparison means much; the settings are the app's.

| | Before | After |
|---|---:|---:|
| Loading the extension | 175 ms | 74 ms |
| First fillet, cut and STEP export, in cells | 2,040 ms | 255 ms |
| Together | 2,205 ms | 330 ms |
| The same again | 30 ms | 30 ms |
| Methods compiled by the JIT in that first operation | 4,547 | 2,640 |

**ReadyToRun is really used.** The runtime's own events count 5,030 of Kerf's methods entered through ReadyToRun after, and none before. In the rig's rows, where only Kerf's load differs: 1,440 methods compiled by the JIT from files, and 1,428 with everything loaded normally, so nothing is being rejected. What remains compiled looks mostly like generics across Kerf's assemblies, which ReadyToRun leaves out since decision 1890, with the extension's own code (inferred from a sample of the names). Loaded normally takes about 210 ms at the same load, against the 170 ms quiet figure, so much of the harness's 330 ms is likely the machine's load; the clean run will say.

## What it costs, which the owner should weigh

- **Kerf doesn't unload, as agreed.** Once loaded it stays until Bento closes, at about 23 MB of private memory (27 to 50 MB), measured after turning the extension off, with the files mapped besides. That's less than it used to take while in use: 130 MB, and about 100 MB just after turning off.
- **An update that brings another Kerf keeps the old one in memory too**, until Bento closes. No restart is needed for the update to work.
- **48 MB more on disk.** Bento loads a copy, in `.shared` under the extension store. The first load of a build of Kerf makes it, about 400 ms once on this machine. A copy unused for 30 days is removed the next time Bento loads Kerf.

## The answers

- **What loads where.** The extension stays in its collectible context, and unloads: a test shows it collected after using Kerf. `Kerf.*.dll` in its folder goes to the shared context. Everything else resolves as before, so Schematiq.Collections, Avalonia and Bento's own assemblies are one copy, from the host. Kerf references only .NET and itself (a test checks), so nothing else had to move.
- **Updating, turning off, removing.** All still work while Bento runs. A loaded file stays open, which is why Bento loads a copy: the extension's own folder is never held. A test removes the extension, finds its folder entirely gone, and installs it again. Without the copy, installing the same version again would have failed on the open files, and a new version carrying the same Kerf could have lost files not yet loaded.
- **A different Kerf: kept apart, never mixed.** There's one context for each build, told apart by each assembly's module id, which the compiler derives from the code, not by version number. So an extension carrying another build, even numbered the same, gets its own copy and context, and a test runs two side by side. Decision 1890's check shows two ReadyToRun copies in a process are safe. Values from one copy can't pass to an extension on the other: a cast fails. That's as it is today, where every extension has a copy of its own.
- **Any extension?** Any that carries Kerf, without declaring anything, and no others: they load and unload exactly as before. The prefix `Kerf.` is the whole test. Making it a general manifest setting would be a larger change, and nothing else needs it yet.

## What the owner must check by hand

The app can't be launched from here. In a build of the branch:

1. Open a workbook with 3D models, and see that the first model appears at once: about a quarter of a second, not two.
2. In `%LOCALAPPDATA%\Bento\extensions\.shared` there's one `kerf-0.3.0.0-…` folder of five DLLs, and the Extensions page doesn't list it.
3. Turn 3D models off and on, then remove and reinstall it, with a workbook open: its cells should go to `#NAME?` and come back, with no restart.
4. The Viewer and the STEP dialog still work: the harness and tests used cells only.
5. An installer build, as the tests ran the packages from a test folder rather than the app's.

Publishing a web app with 3D models isn't affected: it takes the assemblies from the extension's folder, which still holds them all, and the extension isn't browser-safe in any case.
