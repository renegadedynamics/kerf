// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;

namespace OcctNet.DrawLite.Tests;

public class DrawInterpreterTests
{
    [Fact]
    public void Substitutes_variables_commands_and_quotes()
    {
        var anOutput = new StringWriter();
        var anInterpreter = new DrawInterpreter(anOutput);

        anInterpreter.Eval("""
            set a 1
            set b "x$a y"
            set c {literal $a [not run]}
            set d [set b]2
            puts "$b|$c|$d|${a}"
            """);

        Assert.Equal("x1 y|literal $a [not run]|x1 y2|1", anOutput.ToString().TrimEnd());
    }

    [Fact]
    public void Handles_comments_semicolons_and_continuations()
    {
        var anInterpreter = new DrawInterpreter();

        string aResult = anInterpreter.Eval("""
            # a comment \
              continued
            set a 1; set b 2
            set c \
                3
            set c
            """);

        Assert.Equal("3", aResult);
        Assert.True(anInterpreter.TryGetVariable("b", out string aB) && aB == "2");
    }

    [Fact]
    public void A_lone_dollar_is_literal()
        => Assert.Equal("cost $ 5$", new DrawInterpreter().Eval("set a \"cost $ 5$\""));

    [Fact]
    public void Viewer_commands_do_nothing()
        => Assert.Equal("", new DrawInterpreter().Eval("pload TOPTEST\ncheckview -display result -2d -path ${imagedir}.png".Replace("${imagedir}", "x")));

    [Fact]
    public void Unknown_commands_are_reported_with_their_line()
    {
        var anError = Assert.Throws<DrawCommandNotFoundException>(() => new DrawInterpreter().Eval("set a 1\nbox b 1 2 3"));

        Assert.Equal("box", anError.Command);
        Assert.Equal(2, anError.Line);
    }

    [Fact]
    public void Registered_commands_receive_substituted_arguments()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.Register("sum", (_, args) => args.Skip(1).Sum(double.Parse).ToString(System.Globalization.CultureInfo.InvariantCulture));

        Assert.Equal("6", anInterpreter.Eval("set x 3\nsum 1 2 $x"));
    }
}
