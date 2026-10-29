// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_MsgFile.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The process-wide table of message texts by keyword, which <see cref="Message_Msg"/> reads.
/// </summary>
/// <remarks>
/// Only the in-memory table is ported: <c>Load</c>, <c>LoadFile</c>, <c>LoadFromEnv</c> and <c>LoadFromString</c>
/// (OCCT's resource files, read from the directories that environment variables name) are left out, so the table
/// holds only what <see cref="AddMsg"/> put there, and a library result never depends on files or the environment.
/// <see cref="AddMsg"/> is internal (process-wide state, decision 415's rule). As in OCCT, <see cref="Msg(string)"/> of
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
