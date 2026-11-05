// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// A development aid: C# stubs (signatures only) of StepData's classes, mapped by the generator's own rules, so the
// generated code can be compiled before StepData is ported (--stubs <dir>). Never committed as source.

using System.Text;

namespace StepGen;

internal static class Stubs
{
    private static readonly string[] Skip = ["Standard_OStream", "Standard_IStream", "StepData_WriterLib", "StepData_FileProtocol", "Interface_Graph", "Interface_CopyTool", "Interface_GeneralLib", "Interface_ReaderLib", "TColStd_", "Dico", "Message_Messenger", "Resource_FormatType", "StepData_Factors", "std::"];

    public static int Write(Db theDb, string theOcct, string theDir)
    {
        Directory.CreateDirectory(theDir);
        foreach (var cls in theDb.Classes.Values.Where(c => c.Package == "StepData").OrderBy(c => c.Name, StringComparer.Ordinal))
        {
            var sb = new StringBuilder();
            sb.Append("#nullable disable\nnamespace Kerf.Engine;\n");
            bool isAbstract = cls.Methods.Any(m => m.IsPure);
            sb.Append("public ").Append(isAbstract ? "abstract " : "").Append("partial class ").Append(cls.Name);
            if (cls.Base != null)
            {
                sb.Append(" : ").Append(cls.Base);
            }

            sb.Append("\n{\n");
            var aSeen = new HashSet<string>();
            foreach (var m in cls.Methods)
            {
                if (m.IsDtor || m.IsDeleted)
                {
                    continue;
                }

                string aAll = m.Origin;
                if (Skip.Any(s => aAll.Contains(s, StringComparison.Ordinal)))
                {
                    continue;
                }

                string aKey = m.SignatureKey(t => theDb.ParamModifier(t) + theDb.Map(t));
                if (!aSeen.Add(aKey))
                {
                    continue;
                }

                var ps = new List<string>();
                for (int k = 0; k < m.Params.Count; k++)
                {
                    var p = m.Params[k];
                    string s = theDb.ParamModifier(p.Type) + theDb.Map(p.Type) + " " + Db.Id(p.Name ?? "a" + k);
                    if (p.Default != null)
                    {
                        string v = string.Join("", p.Default.Select(x => x.Text));
                        v = v switch { "nullptr" => "null", "Standard_True" => "true", "Standard_False" => "false", _ => v };
                        if (v.Contains('(') || v.Contains("::"))
                        {
                            v = "default";
                        }

                        s += " = " + v;
                    }

                    ps.Add(s);
                }

                string aGeneric = m.TemplateParams.Count > 0 ? "<" + string.Join(",", m.TemplateParams) + ">" : "";
                string aWhere = string.Concat(m.TemplateParams.Select(tp => " where " + tp + " : Standard_Transient"));
                if (m.IsCtor)
                {
                    sb.Append("    public ").Append(cls.Name).Append('(').Append(string.Join(", ", ps)).Append(") { }\n");
                    continue;
                }

                string aMods = m.IsStatic ? "static " : m.IsPure ? "abstract " : m.IsOverride ? "override " : m.IsVirtual ? "virtual " : "";
                if (!m.IsPure && !m.IsOverride && !m.IsStatic && theDb.Ancestors(cls.Name, false).Any(a => a.Methods.Any(x => x.SignatureKey(theDb.Map) == m.SignatureKey(theDb.Map) && (x.IsVirtual || x.IsPure))))
                {
                    aMods = "override ";
                }

                string aRet = m.Return!.IsRef && !m.Return.IsConst && theDb.IsRefKind(m.Return.Bare()) ? "ref " + theDb.Map(m.Return) : theDb.Map(m.Return);
                sb.Append("    public ").Append(aMods).Append(aRet).Append(' ').Append(Db.Id(m.Name)).Append(aGeneric).Append('(').Append(string.Join(", ", ps)).Append(')').Append(aWhere);
                sb.Append(m.IsPure ? ";\n" : " => throw new System.NotImplementedException();\n");
            }

            sb.Append("}\n");
            File.WriteAllText(Path.Combine(theDir, cls.Name + ".cs"), sb.ToString());
        }

        foreach (var e in theDb.Enums.Values.Where(e => theDb.Classes.Values.All(c => c.Enums.All(x => x != e))))
        {
            if (!e.Name.StartsWith("StepData_", StringComparison.Ordinal))
            {
                continue;
            }

            File.WriteAllText(Path.Combine(theDir, e.Name + ".cs"),
                "namespace Kerf.Engine;\npublic enum " + e.Name + " { " + string.Join(", ", e.Members.Select(m => m.Name)) + " }\n");
        }

        _ = theOcct;
        return 0;
    }
}
