# Phase 9 plan: the switch-over

## The goal

design.md's phase 9: "build123d.NET moves onto OCCT.NET, op by op through BRep bytes, then drops native OCCT".

Most of that already happened in phases 5 to 8:
- **build123d.NET** compiles against either backend. On OCCT.NET it needs no native code: STEP was the last native call,
  and unit 8X forwarded it.
- **Bento's 3D models extension** runs on OCCT.NET, through a package packed from the checkouts.

What is left is to make that the normal way things are built and shipped:
- the packages on nuget.org;
- Bento building from them;
- OCCT.NET as build123d.NET's default backend, with native OCCT kept only as the oracle;
- checking, across build123d.NET's whole suite, that the two backends agree.

Phase 9 is small next to phase 8. It is mostly packaging, configuration and measurement, and ports little new code.

## 1. What phase 9 starts from

### 1.1 The three repositories

| Repository | State on `main` |
|---|---|
| **OCCT.NET** | Phases 0–8; 62.0% of OCCT's lines; 11,383 tests. Four packages: OcctNet.Foundation, ModelingData, ModelingAlgorithms, DataExchange (`tools/pack-nuget.ps1`). |
| **build123d.NET** | `Build123dBackend` is `Native` by default; `OcctNet` builds the same code on `Build123d.OcctNet`, 4,384 generated forwarding members. Packages: Build123d.OcctNet and Build123d (on OCCT.NET). |
| **Bento** | The 3D models extension references **Build123d.Managed 0.1.0-spike.4**: one package with all six assemblies, packed by `Extensions/Bento.Cad/pack-build123d.ps1` into a folder feed beside the repository. |

### 1.2 The collections package

On 2 November Bento moved its collections library out to its own repository. It is now **Schematiq.Collections
1.0.0-beta.1** on nuget.org, with its types renamed (`ISchematiqList<T>`, `SchematiqList`, ...). Bento shares it with
extensions as a host package.

OCCT.NET and build123d.NET still referenced Bento.Collections 0.4.0. That package was never on nuget.org, and Bento no
longer ships it. Two consequences:
- **The 3D models extension doesn't load.**
- **The `0.1.0-preview.1` packages can't be used.** They depend on a package nobody can restore.

Unit 9A fixes both, so it comes first.

### 1.3 Package metadata

- **OCCT.NET's packages** have no project or repository URL.
- **build123d.NET's** project URL points at Python build123d (gumyr/build123d).
- **Neither repository has a public remote** yet: OCCT.NET's and Bento's `origin` are local bundles.

### 1.4 How far the backends agree today

- **build123d.NET's suite** passes on both backends.
- **Native results compared op by op through BRep bytes** for only part of build123d.NET:
  - the algebra-mode Booleans (phase 6, `Build123dBooleanOracleTests`);
  - phase 7's fillets, offsets, features and healing (`Build123dPhase7OracleTests`);
  - phase 8's STEP writing, byte for byte (unit 8X1).
- **Not compared yet:** the rest of the suite's OCCT calls (sweeps, lofts, projections, meshing, text, the sketch
  operations) are checked only by build123d.NET's own assertions.
- **Speed:** on OCCT.NET the suite takes 4.4× native's time overall (beyond-compatibility.md, item 2). Nobody has
  profiled where that time goes.

### 1.5 The follow-ups carried in

- **Phase 7** (decisions 1164, 1165):
  - the fillet tail: twenty cases, estimated at 0.3–0.5 million tokens, and as much again for extending Bezier faces;
  - five corpus failures not yet triaged that may be port bugs: bug26406, bug23380, bug29712_44, anc101, bug1281.
- **Phase 8** (decision 1296):
  - an OBJ reader (19 corpus cases with data);
  - TKDE's DE_Wrapper and its providers;
  - GD&T, views, materials, notes and SHUOs in the assembly tree;
  - PLY;
  - StepAP209_Construct and StepTidy;
  - native XCAF in build123d.NET's native backend;
  - Bento on the nuget.org packages, which is unit 9C here.

## 2. Design questions

### 2.1 Which backend is the default, and what happens to native

- **build123d.NET's default:** recommended OCCT.NET.
  - Native stays buildable with `-p:Build123dBackend=Native`, for two jobs:
    - the oracle (OCCT.NET's oracle tests and the native recorders need it);
    - the differential runs (2.3).
  - It is no longer packed or published.
- **Native build123d.NET as a product:** beyond-compatibility.md's decision point A asks whether to retire it.
  - Retiring it lets build123d.NET use OCCT.NET's types directly and drop the forwarding layer (item 1 there).
  - Recommended: not in phase 9. Keep native as the reference while the switch-over settles, and make item 1 the start
    of a phase 10.

### 2.2 How Bento consumes the packages

- **Two options:**
  - keep one bundled package (Build123d.Managed), now packed from the published assemblies;
  - reference the published **Build123d** package, which brings Build123d.OcctNet, the four OcctNet packages and
    Schematiq.Collections as NuGet dependencies.
- **Recommended: the published packages.** NuGet resolves the versions, and pack-build123d.ps1 and the folder feed go.
- **Schematiq.Collections stays a host package.** The extension load context gives Bento's copy, as it does now.
  That needs build123d.NET and OCCT.NET to build against the version Bento ships, which unit 9A restores.

### 2.3 The differential run, op by op

design.md asks for the switch "op by op through BRep bytes". The phase 6 and phase 7 captures already do that for
their operations. Unit 9D generalises them:
- **Record:** run build123d.NET's whole suite on the native backend, recording every forwarded call that returns a
  shape, with its inputs as BRep bytes.
- **Replay:** replay the recording on OCCT.NET.
- **Compare:** BRep bytes, with the oracle's usual masks (the Free and Checked flags, canonical order where native
  iterates by address).
- **Attribute:** each difference, as the earlier captures did.

The forwarding layer (`Build123d.OcctNet`) is generated, so the recording hook can be generated with it. It stays out of
the published packages.

### 2.4 Performance

Item 2 of beyond-compatibility.md, the part that keeps compatibility:
- profile build123d.NET's suite and Bento's example workbook on OCCT.NET;
- take the wins that don't change results: allocation, exceptions used as control flow, and copies the profile shows
  are hot.

The oracle and the corpus stay green throughout. Anything that would change results waits for decision point B.

### 2.5 Versions and releases

- **Prerelease labels:** `0.1.0-preview.N` while Bento is the only consumer.
  - The repack after unit 9A is **preview.2**, or preview.1 again if preview.1 was never pushed.
- **`0.1.0` itself:** when Bento ships a release on these packages.
- **One version:** all six packages share it, as now.
- **The collections dependency:** each package depends on the Schematiq.Collections version Bento ships. When Bento
  moves to a new one, the packages are packed again.

## 3. Units

| Unit | Repository | Content | Needs |
|---|---|---|---|
| **9A** Collections | OCCT.NET, build123d.NET | Schematiq.Collections 1.0.0-beta.1 from nuget.org in place of Bento.Collections 0.4.0. Rename the types, drop the local feed and `tools/pack-bento.ps1`, update the docs. **Done on `phase9/collections` and build123d.NET's `occtnet/schematiq`.** | – |
| **9B** Packages | OCCT.NET, build123d.NET | Package metadata: URLs once the repositories are public (question 41); build123d.NET's project URL. A `-Verify` step in `pack-nuget.ps1`: restore the packed set from a clean folder feed and run a smoke test (fillet a box, STEP round trip, glTF). Repack; the owner pushes. | 9A |
| **9C** Bento on nuget.org | Bento | The 3D models extension references Build123d from nuget.org. Retire Build123d.Managed, `pack-build123d.ps1` and `Build123d.props`'s feed. Schematiq.Collections stays a host package. Check the extension's packing and load context, its tests, and the guide's CAD examples. | 9B pushed |
| **9D** Default backend and the differential run | build123d.NET, OCCT.NET | `Build123dBackend` defaults to `OcctNet`; native becomes opt-in and is no longer packed; README and design.md. The whole-suite recording and replay (2.3), as an oracle test in OCCT.NET; differences attributed. | 9A |
| **9E** Profile | OCCT.NET | Profiles of build123d.NET's suite and Bento's example workbook (2.4). A short report in `docs/`, and the wins that keep results, each checked by the oracle. | 9D |
| **9F** Triage | OCCT.NET | Decision 1165's five possible port bugs: fix or attribute each. | – |
| **9G** OBJ reader | OCCT.NET, build123d.NET | Written fresh over the assembly tree, like 8T's writer, with limits and cancellation; `import_obj` in build123d.NET; the 19 corpus cases. Only if question 44 says so. | – |

Everything else in section 1.5 waits until Bento asks for it (question 44).

### Running order

At most three units at once (the owner's limit).

| Step | Units | Notes |
|---:|---|---|
| 1 | 9A, then 9B | 9A is done and needs only its gate; 9B's repack follows straight on |
| 2 | 9C ‖ 9D ‖ 9F | 9C once the owner has pushed; 9D and 9F don't wait |
| 3 | 9E ‖ 9G | 9E profiles the default backend 9D leaves |

## 4. Milestones

1. **Published and consumed (9A to 9C).** The six packages are on nuget.org, and Bento's 3D models extension builds
   and passes its tests from them, with no folder feed and no checkout of build123d.NET or OCCT.NET.
2. **The switch (9D).** OCCT.NET is build123d.NET's default backend. Every shape-returning operation of build123d.NET's
   suite gives the same BRep bytes as native, or its difference is attributed. Native OCCT survives only as the oracle.
3. **Measured (9E).** Where the 4.4× goes, and how much of it the compatible fixes recover.

## 5. Notes for porters

- **The earlier phases' notes apply:** phase8-plan.md section 6 and handover.md's "Conventions that bite".
- **Cross-repository work:** branches or worktrees only, never the main checkouts. The owner merges build123d.NET's and
  Bento's branches.
- **Bento:** its CLAUDE.md rules hold:
  - build with an artifacts path outside the repository;
  - never launch the app or install the MSI;
  - warnings are errors.
- **Pushing:** nothing is pushed to nuget.org or any remote by us. The owner pushes.

## 6. Questions for the owner

40. **The default backend** (2.1). *Recommended:* OCCT.NET becomes build123d.NET's default. Native stays buildable for
    the oracle but is no longer packed or published, and retiring it outright waits for a phase 10.
41. **Package metadata** (1.3). Project and repository URLs for OCCT.NET and build123d.NET, if they're to be public.
    *Recommended:* until then, no URL rather than Python build123d's.
42. **How Bento references build123d.NET** (2.2). *Recommended:* the published Build123d package and its dependencies,
    in place of the bundled Build123d.Managed.
43. **Versions** (2.5). *Recommended:* the repack after 9A is `0.1.0-preview.2` (or preview.1 if it was never pushed),
    and `0.1.0` comes when Bento releases on it.
44. **Which follow-ups join phase 9** (1.5). *Recommended:* the triage of the five possible port bugs (9F) and the OBJ
    reader (9G). The fillet tail, GD&T and views, PLY, TKDE, StepAP209 and StepTidy wait until Bento needs them.
45. **Performance** (2.4). *Recommended:* profile and take only the wins that keep native's results in phase 9. Changes
    that diverge from OCCT go through beyond-compatibility.md's decision point B.
