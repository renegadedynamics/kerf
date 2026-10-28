// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace OcctNet.DrawLite;

/// <summary>An error in a script, or from a command, with the line it happened on.</summary>
public class DrawException(string message, int line = 0) : Exception(line > 0 ? $"line {line}: {message}" : message)
{
    public int Line { get; } = line;
}

/// <summary>A command the interpreter doesn't know. A corpus run counts these to show what to implement next.</summary>
public sealed class DrawCommandNotFoundException(string command, int line) : DrawException($"invalid command name \"{command}\"", line)
{
    public string Command { get; } = command;
}

/// <summary>A Draw command: the interpreter and the command's arguments (after substitution) in, its result out.</summary>
public delegate string DrawCommand(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs);

/// <summary>
/// A small interpreter for OCCT's Draw test scripts, so the port can run OCCT's own test corpus (tests/ in the
/// OCCT source). It does Tcl's substitution and evaluation; commands are registered by name. This is the
/// skeleton: modelling commands (box, bcut, checkprops, ...) are added as the packages they need are ported.
/// </summary>
public sealed class DrawInterpreter
{
    private readonly Dictionary<string, DrawCommand> myCommands = new(StringComparer.Ordinal);
    private readonly Dictionary<string, string> myVariables = new(StringComparer.Ordinal);

    /// <summary>Where <c>puts</c> writes.</summary>
    public TextWriter Output { get; }

    public DrawInterpreter(TextWriter? theOutput = null)
    {
        Output = theOutput ?? TextWriter.Null;
        DrawBuiltins.Register(this);
    }

    public IReadOnlyCollection<string> CommandNames => myCommands.Keys;

    public void Register(string theName, DrawCommand theCommand) => myCommands[theName] = theCommand;

    public bool HasCommand(string theName) => myCommands.ContainsKey(theName);

    public bool TryGetVariable(string theName, out string theValue) => myVariables.TryGetValue(theName, out theValue!);

    public void SetVariable(string theName, string theValue) => myVariables[theName] = theValue;

    public bool UnsetVariable(string theName) => myVariables.Remove(theName);

    /// <summary>Runs a script and returns the result of its last command.</summary>
    public string Eval(string theScript)
    {
        string aResult = "";
        foreach (Command aCommand in TclParser.Parse(theScript))
        {
            var anArgs = new List<string>(aCommand.Words.Count);
            foreach (Word aWord in aCommand.Words)
            {
                anArgs.Add(Substitute(aWord, aCommand.Line));
            }

            if (!myCommands.TryGetValue(anArgs[0], out DrawCommand? aHandler))
            {
                throw new DrawCommandNotFoundException(anArgs[0], aCommand.Line);
            }

            try
            {
                aResult = aHandler(this, anArgs);
            }
            catch (DrawException)
            {
                throw;
            }
            catch (Exception anError)
            {
                throw new DrawException($"{anArgs[0]}: {anError.Message}", aCommand.Line);
            }
        }

        return aResult;
    }

    private string Substitute(Word theWord, int theLine)
    {
        if (theWord.Literal is { } aLiteral)
        {
            return aLiteral;
        }

        var aBuilder = new StringBuilder();
        foreach (WordPart aPart in theWord.Parts)
        {
            switch (aPart)
            {
                case WordPart.Text aText:
                    aBuilder.Append(aText.Value);
                    break;
                case WordPart.Variable aVariable:
                    if (!myVariables.TryGetValue(aVariable.Name, out string? aValue))
                    {
                        throw new DrawException($"can't read \"{aVariable.Name}\": no such variable", theLine);
                    }

                    aBuilder.Append(aValue);
                    break;
                case WordPart.Command aCommand:
                    aBuilder.Append(Eval(aCommand.Script));
                    break;
            }
        }

        return aBuilder.ToString();
    }
}
