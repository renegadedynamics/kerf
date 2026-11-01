// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The process-wide table of message texts by keyword, which <see cref="Message_Msg"/> reads.
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
public static class Message_MsgFile
{
    private static readonly Dictionary<string, string> msgsDataMap = new(StringComparer.Ordinal);
    private static readonly Lock Message_MsgFile_Mutex = new();

    /// <summary>Adds a message to the table, replacing any message with the same keyword. Always true.</summary>
    internal static bool AddMsg(string theKeyword, string theMessage)
    {
        lock (Message_MsgFile_Mutex)
        {
            msgsDataMap[theKeyword] = theMessage;
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
    private static bool getString(char[] theBuffer, ref int thePtr, out string theString, out int theLeftSpaces)
    {
        theString = "";
        theLeftSpaces = 0;
        int anEndPtr = thePtr;
        int aPtr;
        int aLeftSpaces;

        do
        {
            //    Skip whitespaces in the beginning of the string
            aPtr = anEndPtr;
            aLeftSpaces = 0;
            for (;;)
            {
                char aChar = theBuffer[aPtr];
                if (aChar == ' ')
                {
                    aLeftSpaces++;
                }
                else if (aChar == '\t')
                {
                    aLeftSpaces += 8;
                }
                else if (aChar == '\r' || theBuffer[aPtr] == '\n')
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
            for (anEndPtr = aPtr; theBuffer[anEndPtr] != '\0'; anEndPtr++)
            {
                if (theBuffer[anEndPtr] == '\n')
                {
                    if (theBuffer[anEndPtr - 1] == '\r')
                    {
                        anEndPtr--;
                    }
                    break;
                }
            }

        } while (theBuffer[aPtr] == '!');

        //    form the result
        if (aPtr == anEndPtr)
        {
            return false;
        }
        thePtr = anEndPtr;
        if (theBuffer[thePtr] != '\0')
        {
            theBuffer[thePtr++] = '\0';
        }
        int anEnd = Array.IndexOf(theBuffer, '\0', aPtr);
        theString = new string(theBuffer, aPtr, anEnd - aPtr);
        theLeftSpaces = aLeftSpaces;
        return true;
    }

    // Fills the table of messages from the buffer (ending with two NULs).
    private static bool loadFile(char[] theBuffer)
    {
        string aKeyword = "";
        string aMessage = "";
        LoadingState aState = LoadingState.MsgFile_WaitingKeyword;
        int sCurrentString = 0;
        int aFirstLeftSpaces = 0;

        //    Take strings one-by-one; comments already screened
        while (getString(theBuffer, ref sCurrentString, out string aString, out int aLeftSpaces))
        {
            bool isKeyword = (aString[0] == '.');
            switch (aState)
            {
                case LoadingState.MsgFile_WaitingMoreMessage:
                    if (isKeyword)
                    {
                        AddMsg(aKeyword, aMessage); // terminate the previous one
                        //      Pass from here to 'case MsgFile_WaitingKeyword'
                    }
                    else
                    {
                        //      Add another line to the message already in the buffer 'aMessage'
                        aMessage += '\n';
                        aLeftSpaces -= aFirstLeftSpaces;
                        if (aLeftSpaces > 0)
                        {
                            aMessage += new string(' ', aLeftSpaces);
                        }
                        aMessage += aString;
                        break;
                    }
                    goto case LoadingState.MsgFile_WaitingMessage;
                case LoadingState.MsgFile_WaitingMessage:
                    if (!isKeyword)
                    {
                        aMessage = aString;
                        aFirstLeftSpaces = aLeftSpaces; // remember the starting position
                        aState = LoadingState.MsgFile_WaitingMoreMessage;
                        break;
                    }
                    //      Pass from here to 'case MsgFile_WaitingKeyword'
                    goto case LoadingState.MsgFile_WaitingKeyword;
                case LoadingState.MsgFile_WaitingKeyword:
                    if (isKeyword)
                    {
                        // remove the first dot character and all subsequent spaces + right-trim
                        aKeyword = aString.Substring(1).Trim(' ', '\t', '\n', '\v', '\f', '\r');
                        aState = LoadingState.MsgFile_WaitingMessage;
                    }
                    break;
                default:
                    break;
            }
        }
        //    Process the last string still remaining in the buffer
        if (aState == LoadingState.MsgFile_WaitingMoreMessage)
        {
            AddMsg(aKeyword, aMessage);
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
    internal static bool LoadFromString(string theContent, int theLength = -1)
    {
        int aStringSize = theLength >= 0 ? theLength : theContent.Length;
        if (aStringSize <= 0)
        {
            return false;
        }

        char[] aBuffer = new char[aStringSize + 2];
        theContent.CopyTo(0, aBuffer, 0, aStringSize);
        aBuffer[aStringSize + 0] = '\0';
        aBuffer[aStringSize + 1] = '\0';
        return loadFile(aBuffer);
    }

    /// <summary>True if the table has a message for the keyword.</summary>
    public static bool HasMsg(string theKeyword)
    {
        lock (Message_MsgFile_Mutex)
        {
            return msgsDataMap.ContainsKey(theKeyword);
        }
    }

    /// <summary>
    /// The message for the keyword. For an unknown keyword, the text of keyword <c>Message_Msg_BadKeyword</c> (by default
    /// "Unknown message invoked with the keyword ") followed by the keyword, which is then stored under the keyword.
    /// </summary>
    public static string Msg(string theKeyword)
    {
        lock (Message_MsgFile_Mutex)
        {
            // if message is not found, generate error message and add it to the map to minimize overhead
            // on consequent calls with the same key
            if (!msgsDataMap.TryGetValue(theKeyword, out string? aValPtr))
            {
                // text of the error message can be itself defined in the map
                const string aPrefixCode = "Message_Msg_BadKeyword";
                const string aDefPrefix = "Unknown message invoked with the keyword ";
                string aErrorMessage = msgsDataMap.TryGetValue(aPrefixCode, out string? aPrefValPtr) ? aPrefValPtr : aDefPrefix;
                aErrorMessage += theKeyword;
                msgsDataMap[theKeyword] = aErrorMessage;
                aValPtr = aErrorMessage;
            }

            return aValPtr;
        }
    }
}
