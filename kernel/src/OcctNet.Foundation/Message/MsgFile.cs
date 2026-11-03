// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The process-wide table of message texts by keyword, which <see cref="Msg"/> reads.
/// </summary>
/// <remarks>
/// Only the in-memory table is ported: <c>Load</c>, <c>LoadFile</c> and <c>LoadFromEnv</c> (OCCT's resource files,
/// read from the directories that environment variables name) are left out, so the table holds only what
/// <see cref="AddMsg"/> and <see cref="LoadFromString"/> put there, from the texts compiled into the port (decision
/// 8103), and a library result never depends on files or the environment. <see cref="AddMsg"/> and
/// <see cref="LoadFromString"/> are internal (process-wide state, decision 415's rule). As in OCCT, <see cref="Msg(string)"/> of
/// an unknown keyword stores the "Unknown message invoked with the keyword ..." text under that keyword, so that
/// <see cref="HasMsg"/> is true for it afterwards.
/// </remarks>
public static class MsgFile
{
    private static readonly Dictionary<string, string> MsgsDataMap = new(StringComparer.Ordinal);
    private static readonly Lock Message_MsgFile_Mutex = new();

    /// <summary>Adds a message to the table, replacing any message with the same keyword. Always true.</summary>
    internal static bool AddMsg(string keyword, string message)
    {
        lock (Message_MsgFile_Mutex)
        {
            MsgsDataMap[keyword] = message;
        }
        return true;
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

    // Fills the table of messages from the buffer (ending with two NULs).
    private static bool LoadFile(char[] buffer)
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
                        AddMsg(keyword, message); // terminate the previous one
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
            AddMsg(keyword, message);
        }
        return true;
    }

    /// <summary>
    /// Loads the messages from the text of a message file (keywords starting with a dot, each followed by its message
    /// on one or more lines; lines starting with '!' are comments). False if the text is empty.
    /// </summary>
    /// <remarks>
    /// Internal (process-wide state, decision 415's rule): OCCT.NET loads only the message texts compiled into the port
    /// (decision 975), never files or the environment.
    /// </remarks>
    internal static bool LoadFromString(string content, int length = -1)
    {
        int stringSize = length >= 0 ? length : content.Length;
        if (stringSize <= 0)
        {
            return false;
        }

        char[] buffer = new char[stringSize + 2];
        content.CopyTo(0, buffer, 0, stringSize);
        buffer[stringSize + 0] = '\0';
        buffer[stringSize + 1] = '\0';
        return LoadFile(buffer);
    }

    /// <summary>True if the table has a message for the keyword.</summary>
    public static bool HasMsg(string keyword)
    {
        lock (Message_MsgFile_Mutex)
        {
            return MsgsDataMap.ContainsKey(keyword);
        }
    }

    /// <summary>
    /// The message for the keyword. For an unknown keyword, the text of keyword <c>Message_Msg_BadKeyword</c> (by default
    /// "Unknown message invoked with the keyword ") followed by the keyword, which is then stored under the keyword.
    /// </summary>
    public static string Msg(string keyword)
    {
        lock (Message_MsgFile_Mutex)
        {
            // if message is not found, generate error message and add it to the map to minimize overhead
            // on consequent calls with the same key
            if (!MsgsDataMap.TryGetValue(keyword, out string? valPtr))
            {
                // text of the error message can be itself defined in the map
                const string prefixCode = "Message_Msg_BadKeyword";
                const string defPrefix = "Unknown message invoked with the keyword ";
                string errorMessage = MsgsDataMap.TryGetValue(prefixCode, out string? prefValPtr) ? prefValPtr : defPrefix;
                errorMessage += keyword;
                MsgsDataMap[keyword] = errorMessage;
                valPtr = errorMessage;
            }

            return valPtr;
        }
    }
}
