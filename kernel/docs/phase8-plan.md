# Phase 8 plan: data exchange

*Draft for the owner's review, 1 November 2026. Questions 31 to 39 are at the end and in
[open-questions.md](open-questions.md).*

## The goal

design.md's phase 8: "STEP generated from the EXPRESS schemas, StepData/XSBase/STEPControl ported; immutable assembly
tree; STL, glTF, 3MF, OBJ written fresh." Its milestone is **bytes in and out without native code**.

In practice that means:
- **build123d.NET:** STEP is the last native call its OCCT.NET backend lacks.
- **Bento:** it needs STEP (and STL, then glTF and 3MF) to import and export real CAD files.

## 1. What phase 8 starts from

### 1.1 Inventory

Measured on `main` after phase 7. Physical lines of OCCT V8_0_1, GTests apart. OCCT 8 keeps private headers in `.pxx`
files: 737 of them, about 42,000 lines, almost all in the RWStep* packages. They are counted here.

| Area | Packages (k lines) | Total |
|---|---|---:|
| **STEP's entity classes** (generated in style) | StepBasic 16.9, StepVisual 14.1, StepKinematics 12.0, StepGeom 11.6, StepShape 10.4, StepAP214 8.6, StepRepr 8.3, StepFEA 7.6, StepDimTol 6.0, StepElement 5.2, StepAP203 2.6, StepAP209 2.0, StepAP242 1.0 | 106.4 |
| **Their readers and writers** (RWStep*, generated in style) | RWStepAP214 29.3 (its ReadWriteModule alone 16.3, a hand-kept dispatch of 2,233 cases), RWStepKinematics 21.1, RWStepBasic 19.0, RWStepGeom 16.5, RWStepVisual 15.9, RWStepShape 14.4, RWStepRepr 10.7, RWStepDimTol 10.6, RWStepFEA 8.8, RWStepElement 2.7, RWStepAP203 1.9, RWStepAP242 0.7 | 151.8 |
| **STEP, written by hand** | StepData 13.9; StepFile 5.2 (4.0 of it flex/bison output); HeaderSection, APIHeaderSection, RWHeaderSection 2.9; StepToGeom 3.1; GeomToStep 5.3; StepToTopoDS 7.9; TopoDSToStep 5.5; STEPConstruct 8.2; STEPControl 7.5; STEPCAFControl 15.9; DESTEP 2.0; StepSelect, STEPSelections, STEPEdit 3.9; StepTidy 2.7 | 83.9 |
| **XSBase** (the exchange framework) | IFSelect 29.2, Interface 18.2, Transfer 9.8, XSControl 9.4, MoniTool 4.5, TransferBRep 2.3, XSMessage 2.1, IFGraph 1.8, XSAlgo 1.5, LibCtl 0.2 | 78.9 |
| **XCAF and the OCAF it needs** | TKXCAF 25.8; TKLCAF 30.5, TKCAF 18.6, TKCDF 15.1, TKVCAF 6.2 | 96.2 |
| **Meshes** | RWStl, StlAPI, DESTL 2.7; RWGltf 10.0; RWObj 3.5; RWPly 1.0; RWMesh 3.4; TKDE 4.2 | 24.8 |
| **Excluded by design** | TKDEIGES 141.4, TKDEVRML 22.9 | |

That makes about 345,000 lines for STEP with XSBase, without XCAF.

- **About 258,000 of those lines are the generated-style entities, readers and writers.** They are regular enough to
  generate (section 2.1).
- **The hand-written code is about 160,000 lines:** STEP's 84,000 and XSBase's 79,000. Some of IFSelect may be
  trimmable.

### 1.2 What OCCT's STEP depends on

- **STEPControl and everything below it need no OCAF.** That covers StepData, StepToTopoDS, TopoDSToStep,
  StepToGeom, GeomToStep and STEPConstruct.
  - The one exception is a value class in STEPConstruct_RenderingProperties.
  - STEPControl alone reads and writes shapes and assemblies (as located compounds), but carries no names, colours or
    layers.
- **STEPCAFControl adds the rest** by reading and writing an XCAF document: names, colours, layers, materials,
  validation properties, views, GD&T and metadata.
  - Its XCAF use is concentrated: `TDF_Label` (100 uses in the reader, 137 in the writer), `XCAFDoc_ShapeTool`, and
    the Color, Layer, Material and DimTol tools.
  - XCAF needs about 96,000 lines of OCAF, and through XCAFPrs and XCAFDoc_VisMaterial also parts of the viewer
    (Graphic3d, Image). design.md excludes both.
- **OCCT's glTF, OBJ and PLY readers and writers go through XCAF too,** and glTF needs RapidJSON. STL doesn't.
  OCCT has no 3MF code.
- **Already ported, or small:**
  - `Message_ProgressRange`/`Scope` (52 uses);
  - `Resource_Manager`: STEP's operator sequences come from `resources/XSTEPResource/STEP`;
  - ShapeProcess and ShapeFix (phase 7);
  - `UnitsMethods`: 336 lines, needed, not ported.
- **Process-wide state:**
  - `Interface_Static` parameters (14 files);
  - static protocol, library and controller registries behind mutexes;
  - `StepFile_Read`'s global mutex, which is there because the flex/bison parser isn't reentrant.

### 1.3 Who uses what

- **build123d.NET** has only BRep text input and output today.
  - Its design plans STEP and STL "through streams" (its phase 4.6), and SVG, DXF, 3MF and glTF later (its phase 6).
  - Its notes for Bento (`bento-integration.md` §7) ask for:
    - streams or bytes;
    - BREP, STEP and STL first, glTF later;
    - limits for hostile input;
    - cancellation.
  - It binds only `STEPControl_Reader`/`Writer` natively. Its OCCT.NET backend lists both as "not in OCCT.NET".
- **Bento** reads STL itself (`CAD_STL`, `Stl.cs`) and has its own immutable assembly tree (`CadModel`: parts with
  meshes, colours and alpha; groups). Its CAD design defers STEP to this phase. Exports go through Bento's file
  actions, which need approval.

### 1.4 The corpus

Measured with `DrawCorpus.Scan` over all 21,068 cases. "With data" means the files a case reads are in the public
dataset; that part is a heuristic.

| Blocked only by data-exchange commands | Cases | With data |
|---|---:|---:|
| All (data exchange, XDE and OCAF together) | 2,810 | 1,336 |
| …without XDE documents (`stepread`, `testreadstep`, `readstl`…) | 461 | 112 |
| …needing XDE documents (`XNewDoc`, `ReadStep`, `WriteStep`, `XGetOneShape`, `XStat`…) | 2,349 | 1,224 |
| …of which IGES or VRML (out of scope) | 707 | 419 |

- **The `de` group (1,084 cases, 258 with data) runs entirely through XDE documents.** Its end script reads, checks
  LABELS, COLORS and LAYERS, writes, and reads back. Most of its STEP files aren't public.
- **`xcaf` (521 cases, 379 with data)** needs XDE's colour and layer commands, and 120 of its cases need OCAF storage
  (`SaveAs`/`Open`).
- **`bugs/step` (201)** has 128 cases needing no document.

So the corpus milestone depends on question 31: without an XDE stand-in, phase 8 reaches about 461 cases. With one, up
to about 1,900 (STEP and mesh cases, IGES and OCAF storage apart).

## 2. Design questions

### 2.1 The entity classes: port, generate, or generate from EXPRESS

design.md said "STEP generated from the EXPRESS schemas". But the schemas and OCCT's generator (ExpToCas) were moved
out of OCCT's repository, and the classes have been edited by hand since:
- complex entities written beforehand;
- SELECT types and dispatch modules kept by hand;
- OCCT 8's migration rewrite.

Only 506 of the files still carry a generator stamp. So a generator run from EXPRESS would not reproduce OCCT 8's
classes, and the port's STEP files would differ from native's.

| | A. Port by hand | B. Generate from OCCT's own C++ | C. Generate from EXPRESS |
|---|---|---|---|
| Work | About 258,000 lines, mechanical, about 20 units | A generator over OCCT's entity headers and RW sources, which are very regular (fields, `Init`, accessors; `ReadStep`, `WriteStep`, `Share`). The dispatch modules and the odd classes are done by hand. About 3 units. | A generator from the AP242/AP214 schemas, fetched from outside, plus everything OCCT edited by hand redone |
| Fidelity to native | Class for class | Class for class: generated from OCCT 8's text itself | Differs wherever OCCT's classes differ from the schema |
| Tokens (rough) | 3–4 million | 0.6–1 million | 1–1.5 million, plus reconciliation |

**Recommendation: B.** The generator reads OCCT's `.hxx`/`.pxx`/`.cxx` for the entity and RW packages and emits C#
following the port's rules. Each generated class is checked by:
- compiling it;
- round-tripping the corpus's STEP files through it;
- comparing with native (section 3).

Classes the generator can't handle are ported by hand and listed. design.md's wording becomes "generated from OCCT's
own entity classes" (question 32).

### 2.2 Assemblies, names and colours without OCAF

design.md rules out OCAF and asks for an immutable assembly tree. The options:

| | A. Port XCAF and OCAF | B. STEPCAFControl over an immutable assembly tree | C. STEPControl only |
|---|---|---|---|
| Content | TKXCAF, TKLCAF, TKCAF, TKCDF, TKVCAF (96,000 lines), and parts of Graphic3d and Image | An `XdeTree` (or similar): an immutable tree of products and instances, each node with a shape, a location, a name, a colour and a layer, and later materials and validation properties. STEPCAFControl's reader and writer are ported against it in place of TDF labels and XCAFDoc tools; STEPConstruct is already free of XCAF. | Shapes and assemblies as located compounds; no names, colours or layers |
| Size | About 110,000 lines | About 16,000 lines adapted, plus about 2,000 new | None extra |
| Fidelity | Exact | STEP output equals native's for what the tree carries; GD&T, views and SHUOs come later or never | Exact for what it carries |
| Corpus | All XDE cases | Most XDE cases, through Draw-lite commands over the tree (2.6) | 461 cases |
| Fits the design | No | Yes | Yes, but Bento loses names and colours |

**Recommendation: B, in two steps:**
1. STEPControl first (C), which already gives build123d.NET and Bento shapes and assemblies.
2. Then STEPCAFControl over the immutable tree, for names, colours and layers.

Bento's `CadModel` already has the same shape, so it maps directly. GD&T, kinematics, FEA and views are read into the
entity model (the generated classes cost little) but not transferred into the tree, unless asked (question 33).

### 2.3 The parser

OCCT reads STEP files with a flex/bison parser (`step.lex`, `step.yacc`: 369 lines of grammar; 4,038 lines of
generated C). That parser isn't reentrant, hence `StepFile_Read`'s global mutex.

**Recommendation:** a hand-written C# recursive-descent parser following `step.yacc` (ISO 10303-21). It:
- is reentrant, so there is no global mutex;
- reads from a stream;
- has limits for hostile input: depth, entity count and string length.

Its output (`StepData_StepReaderData`) must equal what OCCT's parser fills in, and the corpus's STEP files check that.
Porting the flex/bison tables literally is the fallback (question 35).

### 2.4 Text encodings and units

- **Encodings:** `StepData_StepReaderData` decodes `\X2\`, `\X4\`, `\S\` and code pages through `Resource_Unicode`,
  which has 25,700 lines of GB2312, Shift-JIS, GBK and Big5 tables. **Recommendation:** .NET's `Encoding` (with
  `CodePagesEncodingProvider`) in their place, checked against OCCT's tables on every code point the corpus uses
  (question 36).
- **Units:** `UnitsMethods` is ported as is.

### 2.5 Meshes: STL ported, the rest written fresh

- **STL:** `RWStl`, `StlAPI` and `DESTL` (2,700 lines) have no XCAF dependency, so they are ported and compared with
  native.
- **glTF, OBJ, 3MF and PLY** are written fresh, as design.md says, over the immutable assembly tree and
  `Poly_Triangulation`. OCCT's versions need XCAF, the viewer's material classes and RapidJSON, and OCCT has no 3MF at
  all.
  - glTF uses `System.Text.Json`.
  - 3MF is a zip of XML.
  - Readers are optional: STL and glTF first (question 34).
- **Comparison:** each is compared with native where native exists (STL; glTF through a recorder), and otherwise
  validated against the format's reference validator or loader (the Khronos glTF validator, the 3MF consortium's
  checks), plus round-trip tests.

### 2.6 Draw-lite and the corpus

- **No documents needed:** `stepread`, `stepwrite`, `testreadstep`, `testwritestep`, `tpcompound`, `readstl`,
  `writestl` and the parameter commands (`param`).
- **XDE commands over the immutable tree** (question 38): `XNewDoc`, `ReadStep`, `WriteStep`, `XGetOneShape`,
  `XStat`, `XGetLengthUnit`, and the colour, layer and label commands the `de` and `xcaf` groups check (`XShow` apart).
  A Draw-lite "document" is an assembly tree under a name. This makes most of `de` and `xcaf` runnable, though not
  OCAF storage (`SaveAs`/`Open`).
- Each stage's commands go with its last unit, as in phase 7.

### 2.7 Process-wide state, threading, streams and limits

- **Parameters:** `Interface_Static` parameters become a parameter object passed to each read and write. Draw-lite
  keeps one per interpreter, so `param` still works.
  - The registries (protocols, libraries, controllers) are initialised once, with no global state that changes.
  - That follows phase 7's handling of ShapeProcess and Resource_Manager.
- **Streams:** every reader and writer takes a stream. File paths are only a convenience on top.
- **Limits and cancellation:**
  - limits for hostile input: file size, entity count, nesting depth;
  - cancellation through `Message_ProgressRange`, which is already ported;
  - that covers bento-integration.md §7's "Should".
- **Results:** STEP readers publish frozen shapes; writers never change their input.

### 2.8 IFSelect

IFSelect (29,000 lines) is XSBase's selection and editing framework, much of it serving Draw. **Recommendation:** port
what STEPControl, XSControl and the Draw-lite commands reach, by a member trace as 7L did. Leave out the rest, and
record the cut.

## 3. The oracle

**Bound natively today:** `STEPControl_Reader`/`Writer`. To add on a build123d.NET branch (question 27 allows it):
- `Interface_Static` (to fix the header);
- `STEPCAFControl_Reader`/`Writer` with a minimal XCAF document wrapper (native's XCAF DLLs are built);
- `RWStl`/`StlAPI`.

Native recorders (`tools/steprec`) cover what can't be bound.

**The comparisons:**
- **Reading:** the same STEP bytes into native and the port. Compare the transferred shapes as BRep bytes, as phase 6
  did, and the assembly tree (names, colours, layers, locations) as text.
- **Writing:** the same shape into both writers. Compare the STEP text with the FILE_NAME line masked: native stamps the
  local time and the path there. Entity numbering follows the order entities are added, which should be deterministic.
  If numbering differs, compare the parsed entity graphs.
- **Round trips:** port write → native read, and native write → port read.
- **Hand-written STEP:** the corpus's STEP files, plus build123d.NET's and Bento's examples written to STEP.

## 4. Units

Sizes are thousands of OCCT lines, or of new C# where marked.

| Unit | Content | Size | Needs |
|---|---|---:|---|
| **8A** XSBase | Interface, Transfer, TransferBRep, XSControl, XSAlgo, MoniTool, XSMessage, IFGraph, LibCtl; IFSelect as traced (2.8); the parameter object (2.7) | 45–79 | phase 7 |
| **8B** StepData and the parser | StepData, the C# parser (2.3), HeaderSection, APIHeaderSection, RWHeaderSection, text encodings (2.4), UnitsMethods | 17 + parser | 8A |
| **8C1** The entity generator | The generator over OCCT's entity and RW sources (2.1), and its output for StepBasic, StepGeom, StepShape and StepRepr with their RW packages | ≈ 3 (C#) + 100 generated | 8B |
| **8C2** The other entities | StepVisual, StepAP214/203/209/242, StepDimTol, StepKinematics, StepFEA, StepElement, and their RW packages; the dispatch modules (`RWStepAP214_ReadWriteModule`, `GeneralModule`, `StepAP214_Protocol`) | 158 generated, about 26 by hand | 8C1 |
| **8D** Geometry | StepToGeom, GeomToStep | 8.4 | 8C1 |
| **8E** Topology | StepToTopoDS, TopoDSToStep | 13.4 | 8D |
| **8F** STEPControl | STEPControl, STEPConstruct, DESTEP's parameters, StepSelect and STEPSelections as needed; the no-document Draw commands | 18–22 | 8C2, 8E |
| **8G** The assembly tree and STEPCAFControl | The immutable tree (2.2), STEPCAFControl over it (names, colours, layers; materials and validation properties if question 33 says so); the XDE Draw commands over the tree (2.6) | ≈ 2 new + 16 adapted | 8F |
| **8H** Meshes | RWStl, StlAPI, DESTL ported; glTF, OBJ, 3MF (and PLY) written fresh over the tree (2.5) | 2.7 + ≈ 4 new | 8G (tree), phase 4 |
| **8W** Corpus | The `de`, `xcaf`, `bugs/step`, `de_mesh` and `gdt` grids' runs and attributions; the cheap Draw gaps the scan shows | ≈ 2 (C#) | 8F, 8G, 8H |
| **8X** build123d.NET and Bento | build123d.NET: `import_step`/`export_step` (with labels, colours and assemblies), STL, glTF, 3MF, through streams with limits and cancellation; its OCCT.NET backend forwards STEP. Bento: `CAD_IO_*` functions (import as a cell value; export as an approved file action). Native bindings for the oracle (3). | ≈ 3 (C#) | 8F, 8G, 8H |

**Totals:** about 345,000 OCCT lines (258,000 of them generated), plus about 15,000 lines of new C#. That is about
the size of phases 5 and 6 together. Because of the generator, the token cost is closer to phase 7's.

### The critical path and the running order

- **The critical path:** 8A → 8B → 8C1 → 8C2 → 8F → 8G → 8X. 8D and 8E run beside 8C2; 8H beside 8G.
- **Running order,** with at most three units at once (the owner's limit since phase 7):

| Step | Units | Notes |
|---:|---|---|
| 1 | 8A | IFSelect's member trace first |
| 2 | 8B ‖ the generator's design (8C1's first half) | The generator needs StepData's types, so it starts once 8B's interfaces are fixed |
| 3 | 8C1 | Generator and the core entities; the pilot for the generated code's review |
| 4 | 8C2 ‖ 8D | |
| 5 | 8E ‖ 8H (STL) | |
| 6 | 8F | First milestone: shapes and assemblies in and out, compared with native |
| 7 | 8G ‖ 8H (fresh writers) | |
| 8 | 8X ‖ 8W | Milestones 2 and 3 |

## 5. Milestones

1. **After 8F: STEP shapes and assemblies without native code.** STEP files read and written byte for byte as native,
   FILE_NAME apart, or as the same entity graph, and shapes compared as BRep bytes.
2. **After 8X: build123d.NET's and Bento's import and export.** build123d.NET's OCCT.NET backend forwards STEP, so its
   last native call is gone; its import and export functions exist; Bento gets `CAD_IO_*`. That completes design.md's
   phase 9 switch-over for build123d.NET, NuGet publishing apart (question 39).
3. **After 8W: the corpus.** The data-exchange cases, at DRAWEXE's rate (OCCT's TODOs apart). That is about 461 cases
   without the XDE stand-in, up to about 1,900 with it, IGES and OCAF storage excluded.

## 6. Notes for porters

- **The earlier phases' notes apply:** phase7-plan.md section 6, and handover.md's "Conventions that bite".
- **OCCT 8's `.pxx` private headers** are part of the source: read them.
- **Generated code is reviewed like ported code:** the generator emits `// Port of:` lines and keeps OCCT's names.
  Hand-edited classes are listed in its report.
- **Determinism:** the writer's entity order must not depend on hashing. Where OCCT iterates an address-keyed map
  while adding entities, the port keeps insertion order and logs the site.
- **Native oracle loops:** `GC.KeepAlive` after every native `TopoDS_Iterator` loop (decision 1109). build123d.NET's
  wrapper now does it too.

## 7. Questions for the owner

31. **Assemblies, names and colours without OCAF** (2.2). *Recommended:* STEPControl first, then STEPCAFControl over
    an immutable assembly tree. Not porting XCAF and OCAF.
32. **The entity classes** (2.1). *Recommended:* generate them from OCCT's own C++ rather than port them by hand or
    regenerate them from EXPRESS. design.md's "generated from the EXPRESS schemas" becomes "generated from OCCT's
    entity classes".
33. **STEP content beyond shapes, names, colours and layers:** materials, validation properties, GD&T/PMI, views,
    kinematics, FEA, metadata. *Recommended:* parse everything (generated classes are cheap), but transfer only
    shapes, assemblies, names, colours, layers, and validation properties. The rest when Bento needs it.
34. **Mesh formats** (2.5). *Recommended:* STL ported; glTF, OBJ and 3MF writers written fresh; readers for STL and
    glTF; PLY only if wanted.
35. **The STEP parser** (2.3). *Recommended:* a hand-written, reentrant C# parser with limits, rather than OCCT's
    flex/bison tables.
36. **Text encodings** (2.4). *Recommended:* .NET's encodings, checked against OCCT's tables, rather than porting
    `Resource_Unicode`'s 25,700 lines of tables.
37. **IFSelect** (2.8). *Recommended:* port only what a member trace reaches.
38. **XDE Draw commands over the immutable tree** (2.6), so most of the `de` and `xcaf` grids run. *Recommended:* yes,
    in 8G.
39. **Publishing:** whether phase 8 ends with build123d.NET and OCCT.NET as NuGet packages in place of Bento's folder
    feed (Bento's design doc, section 12). *Recommended:* yes, as the last step of 8X, once you choose package names and
    a feed (nuget.org or private).
