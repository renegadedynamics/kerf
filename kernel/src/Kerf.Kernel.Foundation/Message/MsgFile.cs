// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The table of message texts by keyword, which <see cref="Msg"/> reads.
/// </summary>
/// <remarks>
/// <para>
/// Only the in-memory table is ported: <c>Load</c>, <c>LoadFile</c> and <c>LoadFromEnv</c> (OCCT's resource files,
/// read from the directories that environment variables name) are left out, so a library result never depends on files
/// or the environment (decision 975).
/// </para>
/// <para>
/// The table has two layers (unit 10I; OCCT's is one process-wide map behind a mutex):
/// </para>
/// <list type="bullet">
/// <item><description><b>The defaults:</b> the texts compiled into the kernel (the Booleans', Shape Healing's, data
/// exchange's), each set added once by its owner's one-time initialisation (<see cref="AddDefaults(string)"/>). An immutable
/// table, replaced whole when a set is added, so readers take no lock.</description></item>
/// <item><description><b>The flow's additions:</b> <see cref="AddMsg"/> and <see cref="LoadFromString"/> (internal:
/// Draw-lite and tests) add to the current <see cref="OperationContext"/> (<see cref="AdditionsSetting"/>), over the
/// defaults, for the rest of the flow of execution.</description></item>
/// </list>
/// <para>
/// <see cref="Msg(string)"/> of an unknown keyword gives OCCT's "Unknown message invoked with the keyword ..." text.
/// OCCT also stores it under the keyword, so that <see cref="HasMsg"/> is true for it afterwards in the whole process;
/// the port doesn't: a read never writes the table.
/// </para>
/// </remarks>
internal static class MsgFile
{
    // The defaults: the compiled-in texts, added once per set by AddDefaults
    private static ImmutableDictionary<string, string> s_defaults = ImmutableDictionary.Create<string, string>(StringComparer.Ordinal);

    /// <summary>The texts the current flow of execution added over the defaults.</summary>
    internal static readonly ContextKey<ImmutableDictionary<string, string>> AdditionsSetting =
        new("MsgFile.Additions", ImmutableDictionary.Create<string, string>(StringComparer.Ordinal));

    /// <summary>
    /// Adds a message to the current flow's table, replacing any message with the same keyword (Draw-lite and tests).
    /// Always true.
    /// </summary>
    internal static bool AddMsg(string keyword, string message)
    {
        OperationContext.Set(AdditionsSetting, AdditionsSetting.Value.SetItem(keyword, message));
        return true;
    }

    /// <summary>
    /// Adds a set of compiled-in texts (a message file's text, as <see cref="LoadFromString"/> reads it) to the
    /// defaults: for the kernel's one-time initialisations, each with its own keywords. False if the text is empty.
    /// </summary>
    internal static bool AddDefaults(string content)
    {
        if (content.Length == 0)
        {
            return false;
        }
        AddDefaults(Parse(content));
        return true;
    }

    /// <summary>Adds a set of compiled-in texts, keyword and text, to the defaults (see <see cref="AddDefaults(string)"/>).</summary>
    internal static void AddDefaults(IEnumerable<KeyValuePair<string, string>> messages)
    {
        ImmutableDictionary<string, string> current, updated;
        do
        {
            current = Volatile.Read(ref s_defaults);
            updated = current.SetItems(messages);
        }
        while (!ReferenceEquals(Interlocked.CompareExchange(ref s_defaults, updated, current), current));
    }

    private enum LoadingState
    {
        MsgFile_WaitingKeyword,
        MsgFile_WaitingMessage,
        MsgFile_WaitingMoreMessage,
        MsgFile_Indefinite
    }

    // Takes a string from the buffer: strings are left-trimmed; those beginning with '!' are omitted. As OCCT's, it
    // ends the string in the buffer with a NUL.
    private static bool GetString(char[] buffer, ref int ptr, out string theString, out int leftSpaces)
    {
        theString = "";
        leftSpaces = 0;
        int endPtr = ptr;
        int aPtr;
        int aLeftSpaces;

        do
        {
            //    Skip whitespaces in the beginning of the string
            aPtr = endPtr;
            aLeftSpaces = 0;
            for (;;)
            {
                char aChar = buffer[aPtr];
                if (aChar == ' ')
                {
                    aLeftSpaces++;
                }
                else if (aChar == '\t')
                {
                    aLeftSpaces += 8;
                }
                else if (aChar == '\r' || buffer[aPtr] == '\n')
                {
                    aLeftSpaces = 0;
                }
                else
                {
                    break;
                }
                aPtr++;
            }

            //    Find the end of the string
            for (endPtr = aPtr; buffer[endPtr] != '\0'; endPtr++)
            {
                if (buffer[endPtr] == '\n')
                {
                    if (buffer[endPtr - 1] == '\r')
                    {
                        endPtr--;
                    }
                    break;
                }
            }

        } while (buffer[aPtr] == '!');

        //    form the result
        if (aPtr == endPtr)
        {
            return false;
        }
        ptr = endPtr;
        if (buffer[ptr] != '\0')
        {
            buffer[ptr++] = '\0';
        }
        int end = Array.IndexOf(buffer, '\0', aPtr);
        theString = new string(buffer, aPtr, end - aPtr);
        leftSpaces = aLeftSpaces;
        return true;
    }

    // Reads the messages from the buffer (ending with two NULs) into the list, in order.
    private static bool LoadFile(char[] buffer, List<KeyValuePair<string, string>> messages)
    {
        string keyword = "";
        string message = "";
        LoadingState state = LoadingState.MsgFile_WaitingKeyword;
        int sCurrentString = 0;
        int firstLeftSpaces = 0;

        //    Take strings one-by-one; comments already screened
        while (GetString(buffer, ref sCurrentString, out string aString, out int leftSpaces))
        {
            bool isKeyword = (aString[0] == '.');
            switch (state)
            {
                case LoadingState.MsgFile_WaitingMoreMessage:
                    if (isKeyword)
                    {
                        messages.Add(new(keyword, message)); // terminate the previous one
                        //      Pass from here to 'case MsgFile_WaitingKeyword'
                    }
                    else
                    {
                        //      Add another line to the message already in the buffer 'aMessage'
                        message += '\n';
                        leftSpaces -= firstLeftSpaces;
                        if (leftSpaces > 0)
                        {
                            message += new string(' ', leftSpaces);
                        }
                        message += aString;
                        break;
                    }
                    goto case LoadingState.MsgFile_WaitingMessage;
                case LoadingState.MsgFile_WaitingMessage:
                    if (!isKeyword)
                    {
                        message = aString;
                        firstLeftSpaces = leftSpaces; // remember the starting position
                        state = LoadingState.MsgFile_WaitingMoreMessage;
                        break;
                    }
                    //      Pass from here to 'case MsgFile_WaitingKeyword'
                    goto case LoadingState.MsgFile_WaitingKeyword;
                case LoadingState.MsgFile_WaitingKeyword:
                    if (isKeyword)
                    {
                        // remove the first dot character and all subsequent spaces + right-trim
                        keyword = aString.Substring(1).Trim(' ', '\t', '\n', '\v', '\f', '\r');
                        state = LoadingState.MsgFile_WaitingMessage;
                    }
                    break;
                default:
                    break;
            }
        }
        //    Process the last string still remaining in the buffer
        if (state == LoadingState.MsgFile_WaitingMoreMessage)
        {
            messages.Add(new(keyword, message));
        }
        return true;
    }

    /// <summary>
    /// Loads the messages from the text of a message file (keywords starting with a dot, each followed by its message
    /// on one or more lines; lines starting with '!' are comments) into the current flow's table. False if the text is
    /// empty.
    /// </summary>
    /// <remarks>
    /// Internal (Draw-lite and tests): the kernel loads only the message texts compiled into it (decision 975), into
    /// the defaults (<see cref="AddDefaults(string)"/>), never files or the environment.
    /// </remarks>
    internal static bool LoadFromString(string content, int length = -1)
    {
        int stringSize = length >= 0 ? length : content.Length;
        if (stringSize <= 0)
        {
            return false;
        }
        List<KeyValuePair<string, string>> messages = Parse(content, stringSize);
        if (messages.Count > 0)
        {
            OperationContext.Set(AdditionsSetting, AdditionsSetting.Value.SetItems(messages));
        }
        return true;
    }

    // The messages of a message file's text, in order
    private static List<KeyValuePair<string, string>> Parse(string content, int length = -1)
    {
        int stringSize = length >= 0 ? length : content.Length;
        var messages = new List<KeyValuePair<string, string>>();
        if (stringSize <= 0)
        {
            return messages;
        }
        char[] buffer = new char[stringSize + 2];
        content.CopyTo(0, buffer, 0, stringSize);
        buffer[stringSize + 0] = '\0';
        buffer[stringSize + 1] = '\0';
        LoadFile(buffer, messages);
        return messages;
    }

    // The message for the keyword: the flow's, else the default; null if neither has it
    private static string? Find(string keyword)
    {
        if (OperationContext.Current.TryGet(AdditionsSetting, out ImmutableDictionary<string, string> additions)
            && additions.TryGetValue(keyword, out string? added))
        {
            return added;
        }
        return Volatile.Read(ref s_defaults).TryGetValue(keyword, out string? value) ? value : null;
    }

    /// <summary>True if the table has a message for the keyword.</summary>
    public static bool HasMsg(string keyword) => Find(keyword) is not null;

    /// <summary>
    /// The message for the keyword. For an unknown keyword, the text of keyword <c>Message_Msg_BadKeyword</c> (by default
    /// "Unknown message invoked with the keyword ") followed by the keyword (not stored: see the remarks).
    /// </summary>
    public static string Msg(string keyword)
    {
        if (Find(keyword) is { } message)
        {
            return message;
        }
        // text of the error message can be itself defined in the map
        const string prefixCode = "Message_Msg_BadKeyword";
        const string defPrefix = "Unknown message invoked with the keyword ";
        return (Find(prefixCode) ?? defPrefix) + keyword;
    }
}
