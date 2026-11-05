// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>expr: arithmetic and its types, comparisons, logic, functions, and Tcl's number formatting.</summary>
public class TclExprTests
{
    [Theory]
    [InlineData("1 + 2", "3")]
    [InlineData("1.0/3", "0.3333333333333333")]
    [InlineData("2.0*3", "6.0")]
    [InlineData("1.5 * 2", "3.0")]
    [InlineData("10/4", "2")]
    [InlineData("-7/2", "-4")]
    [InlineData("-7%2", "1")]
    [InlineData("7%-2", "-1")]
    [InlineData("10/4.0", "2.5")]
    [InlineData("2**10", "1024")]
    [InlineData("2**-1", "0")]
    [InlineData("2**0.5", "1.4142135623730951")]
    [InlineData("2**3**2", "512")]
    [InlineData("-2**2", "4")]
    [InlineData("1 + 2 * 3 - 4", "3")]
    [InlineData("(1 + 2) * 3", "9")]
    [InlineData("010", "8")]
    [InlineData("0x1F", "31")]
    [InlineData("0b101", "5")]
    [InlineData("0o17", "15")]
    [InlineData("1.e+05", "100000.0")]
    [InlineData(".5", "0.5")]
    [InlineData("1e20", "1e+20")]
    [InlineData("1e16", "10000000000000000.0")]
    [InlineData("1e17", "1e+17")]
    [InlineData("123456789012345678.0", "1.2345678901234568e+17")]
    [InlineData("0.0001", "0.0001")]
    [InlineData("0.00001", "1e-5")]
    [InlineData("1.5e-7", "1.5e-7")]
    [InlineData("100.0", "100.0")]
    [InlineData("0.1 + 0.2", "0.30000000000000004")]
    [InlineData("-0.5", "-0.5")]
    [InlineData("1.0/0", "Inf")]
    [InlineData("-1.0/0", "-Inf")]
    [InlineData("9223372036854775807", "9223372036854775807")]
    [InlineData("-9223372036854775808", "-9223372036854775808")]
    public void Arithmetic_follows_tcl_types_and_formatting(string theExpression, string theExpected)
        => Assert.Equal(theExpected, Tcl.Eval($"expr {{{theExpression}}}"));

    [Theory]
    [InlineData("\"abc\" < \"abd\"", "1")]
    [InlineData("\"10\" < \"9\"", "0")]
    [InlineData("\"10\" < \"9a\"", "1")]
    [InlineData("1 == 1.0", "1")]
    [InlineData("\"1\" eq \"1.0\"", "0")]
    [InlineData("\"a\" ne \"b\"", "1")]
    [InlineData("[list a b] eq \"a b\"", "1")]
    [InlineData("\"b\" in {a b c}", "1")]
    [InlineData("\"d\" in {a b c}", "0")]
    [InlineData("\"d\" ni {a b c}", "1")]
    [InlineData("\"\" == \"\"", "1")]
    [InlineData("\" 12 \" + 1", "13")]
    [InlineData("\"0x10\" + 1", "17")]
    [InlineData("1 < 2 && 3 < 4", "1")]
    [InlineData("0 || 0.0", "0")]
    [InlineData("!0", "1")]
    [InlineData("!\"false\"", "1")]
    [InlineData("!yes", "0")]
    [InlineData("true && on", "1")]
    [InlineData("1 ? \"yes\" : \"no\"", "yes")]
    [InlineData("0 ? 1 : 0 ? 2 : 3", "3")]
    [InlineData("5 & 3", "1")]
    [InlineData("5 | 3", "7")]
    [InlineData("5 ^ 3", "6")]
    [InlineData("~5", "-6")]
    [InlineData("1 << 4", "16")]
    [InlineData("-16 >> 2", "-4")]
    [InlineData("3 > 2 == 1", "1")]
    [InlineData("\"abc\"", "abc")]
    [InlineData("\"1.50\"", "1.5")]
    [InlineData("{abc}", "abc")]
    [InlineData("Inf > 1e308", "1")]
    public void Comparisons_logic_and_bit_operators(string theExpression, string theExpected)
        => Assert.Equal(theExpected, Tcl.Eval($"expr {{{theExpression}}}"));

    [Theory]
    [InlineData("abs(-3)", "3")]
    [InlineData("abs(-3.5)", "3.5")]
    [InlineData("abs (-2)", "2")]
    [InlineData("sqrt(16)", "4.0")]
    [InlineData("pow(2,3)", "8.0")]
    [InlineData("exp(0)", "1.0")]
    [InlineData("log(1)", "0.0")]
    [InlineData("log10(1000)", "3.0")]
    [InlineData("sin(0)", "0.0")]
    [InlineData("cos(0)", "1.0")]
    [InlineData("tan(0)", "0.0")]
    [InlineData("asin(1)", "1.5707963267948966")]
    [InlineData("acos(1)", "0.0")]
    [InlineData("atan(1)", "0.7853981633974483")]
    [InlineData("atan2(1, 1)", "0.7853981633974483")]
    [InlineData("int(3.7)", "3")]
    [InlineData("int(-3.7)", "-3")]
    [InlineData("int(5)", "5")]
    [InlineData("round(2.5)", "3")]
    [InlineData("round(-2.5)", "-3")]
    [InlineData("round(2.4)", "2")]
    [InlineData("floor(-1.5)", "-2.0")]
    [InlineData("ceil(1.2)", "2.0")]
    [InlineData("double(3)", "3.0")]
    [InlineData("double(1)/3", "0.3333333333333333")]
    [InlineData("min(3, 1, 2)", "1")]
    [InlineData("max(1, 2.5)", "2.5")]
    [InlineData("hypot(3, 4)", "5.0")]
    [InlineData("fmod(7, 3)", "1.0")]
    [InlineData("fmod(-7, 3)", "-1.0")]
    [InlineData("wide(7.9)", "7")]
    [InlineData("entier(-0.5)", "0")]
    [InlineData("bool(5)", "1")]
    public void Math_functions(string theExpression, string theExpected)
        => Assert.Equal(theExpected, Tcl.Eval($"expr {{{theExpression}}}"));

    [Fact]
    public void Variables_and_commands_are_substituted_inside_braced_expressions()
    {
        Assert.Equal("6", Tcl.Eval("set x 3; expr {$x * 2}"));
        Assert.Equal("4", Tcl.Eval("expr {[llength {a b c}] + 1}"));
        Assert.Equal("10", Tcl.Eval("set a(1) 5; set i 1; expr {$a($i) * 2}"));
        Assert.Equal("1", Tcl.Eval("set s {}; expr {$s == \"\"}"));
        Assert.Equal("1", Tcl.Eval("set s abc; expr {\"$s\" == \"abc\"}"));
        Assert.Equal("1", Tcl.Eval("set s abc; expr {\"[string toupper $s]\" eq \"ABC\"}"));
        // braces keep a value an operand; without them its text is parsed as part of the expression
        Assert.Equal("3", Tcl.Eval("set x 1+1; expr $x*2"));
        Assert.Equal("can't use non-numeric string as operand of \"*\"", Tcl.Error("set x 1+1; expr {$x*2}"));
        Assert.Equal("1", Tcl.Eval("expr 3 > 2"));
        Assert.Equal("0.1", Tcl.Eval("set v 1e-1; expr $v"));
    }

    [Fact]
    public void Short_circuit_skips_the_other_operand()
    {
        Assert.Equal("0", Tcl.Eval("expr {0 && [error never]}"));
        Assert.Equal("1", Tcl.Eval("expr {1 || [error never]}"));
        Assert.Equal("ok", Tcl.Eval("expr {1 ? \"ok\" : [error never]}"));
    }

    [Theory]
    [InlineData("\"abc\" + 1", "can't use non-numeric string as operand of \"+\"")]
    [InlineData("\"\" + 1", "can't use empty string as operand of \"+\"")]
    [InlineData("1/0", "divide by zero")]
    [InlineData("1%0", "divide by zero")]
    [InlineData("sqrt(-1)", "domain error: argument not in valid range")]
    [InlineData("1.5 % 1", "can't use floating-point value \"1.5\" as operand of \"%\"")]
    [InlineData("1.5 & 1", "can't use floating-point value \"1.5\" as operand of \"&\"")]
    [InlineData("nosuch(1)", "invalid command name \"tcl::mathfunc::nosuch\"")]
    public void Errors_match_tcl(string theExpression, string theError) => Assert.Equal(theError, Tcl.Error($"expr {{{theExpression}}}"));

    [Fact]
    public void Syntax_errors_are_reported()
    {
        Assert.Contains("syntax error", Tcl.Error("expr {1 +}"));
        Assert.Contains("invalid bareword", Tcl.Error("expr {abc}"));
        Assert.Contains("empty expression", Tcl.Error("expr {}"));
    }

    [Theory]
    [InlineData(0.5, "0.5")]
    [InlineData(1.0, "1.0")]
    [InlineData(-2.0, "-2.0")]
    [InlineData(1e300, "1e+300")]
    [InlineData(1.0 / 3, "0.3333333333333333")]
    [InlineData(2.0 / 3, "0.6666666666666666")]
    [InlineData(1234567.0, "1234567.0")]
    [InlineData(0.001, "0.001")]
    [InlineData(5e-324, "5e-324")]
    [InlineData(double.NaN, "NaN")]
    public void Doubles_print_as_tcl_does(double theValue, string theExpected) => Assert.Equal(theExpected, TclNumber.FormatDouble(theValue));

    [Fact]
    public void Expressions_in_if_while_and_for_are_the_same_language()
    {
        Assert.Equal("small", Tcl.Eval("set v 0.5; if {abs($v) < 1 && $v != 0} {set r small} else {set r big}"));
        Assert.Equal("Error: x = 3 is not equal to expected 2", Tcl.Output("""
            proc checkreal {name value expected tol_abs tol_rel} {
                if { abs ($value - $expected) > $tol_abs + $tol_rel * abs ($expected) } {
                    puts "Error: $name = $value is not equal to expected $expected"
                } else {
                    puts "Check of $name OK: value = $value, expected = $expected"
                }
                return
            }
            checkreal x 3 2 0.1 0.01
            """).TrimEnd());
    }
}
