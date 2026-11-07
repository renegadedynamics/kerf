# Booleans check their result's volume, for Bento's branch

The owner's decision (kernel `docs/decisions.md`, 1860): a Boolean proved wrong fails, as an invalid one does (`11r-bento.md`).

**Nothing has to change for Bento to build or to show the reason.** The code is `result.invalid`, so `ModelFunctions.Run` gives `#VALUE!` with the message, and `ModelingDiagnostics.TryGetInvalid` gives the shape to draw as broken.

| Cell | Before | Now |
|---|---|---|
| A cut that comes out with a solid inside out (a thread at some depths) | A shape with the right total and the wrong answer | `#VALUE!`: "The cut's result is wrong: one of its solids is inside out, with a volume of -47.28; the object's volume is 7871.7, the tool's 88.15 and the result's 7871.7." |
| A fuse larger than its inputs together | A shape | `#VALUE!`: "The fuse's result is wrong: a fuse can't be larger than its inputs together; …" |
| A cut that removed nothing, or a common that's empty, of solids that overlap | The object uncut, or nothing | `#VALUE!`: "The cut's result is wrong: it removed nothing, but the point (…) and others are inside the tool and still in the result." |

Eleven of the guide's examples are unchanged. A workbook that leaned on one of these wrong answers will now show the error, which is the point of it.

**New public API, none of it needed by Bento:** `BooleanVolumes.Check` and `BooleanVolumeRule`, in `Kerf.Booleans`. Bento's own volumes are unaffected: the check keeps nothing on a shape.
