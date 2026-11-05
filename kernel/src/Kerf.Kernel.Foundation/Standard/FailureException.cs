// Port of: src/FoundationClasses/TKernel/Standard/Standard_Failure.hxx
// Port of: src/FoundationClasses/TKernel/Standard/Standard_*.hxx (DEFINE_STANDARD_EXCEPTION)
// Port of: src/FoundationClasses/TKernel/StdFail/StdFail_*.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_VectorWithNullMagnitude.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The root of OCCT's exceptions. OCCT's class hierarchy is kept, so code can catch a whole family
/// (<see cref="DomainErrorException"/>) as in C++.
/// </summary>
internal class FailureException : Exception
{
    public FailureException() { }

    public FailureException(string? message) : base(message) { }

    public FailureException(string? message, Exception? innerException) : base(message, innerException) { }

    /// <summary>The OCCT name of the exception's type, as <c>DynamicType()->Name()</c> gives it.</summary>
    public string OcctType => OcctTypeNames.Of(GetType());

    /// <summary>The message, or an empty string when there is none, as OCCT's <c>GetMessageString</c>.</summary>
    public string GetMessageString() => base.Message;
}

internal class DomainErrorException : FailureException
{
    public DomainErrorException() { }
    public DomainErrorException(string? message) : base(message) { }
}

internal class ConstructionErrorException : DomainErrorException
{
    public ConstructionErrorException() { }
    public ConstructionErrorException(string? message) : base(message) { }
}

internal class DimensionErrorException : DomainErrorException
{
    public DimensionErrorException() { }
    public DimensionErrorException(string? message) : base(message) { }
}

internal class DimensionMismatchException : DimensionErrorException
{
    public DimensionMismatchException() { }
    public DimensionMismatchException(string? message) : base(message) { }
}

internal class ImmutableObjectException : DomainErrorException
{
    public ImmutableObjectException() { }
    public ImmutableObjectException(string? message) : base(message) { }
}

internal class MultiplyDefinedException : DomainErrorException
{
    public MultiplyDefinedException() { }
    public MultiplyDefinedException(string? message) : base(message) { }
}

internal class NoMoreObjectException : DomainErrorException
{
    public NoMoreObjectException() { }
    public NoMoreObjectException(string? message) : base(message) { }
}

internal class NoSuchObjectException : DomainErrorException
{
    public NoSuchObjectException() { }
    public NoSuchObjectException(string? message) : base(message) { }
}

internal class NullObjectException : DomainErrorException
{
    public NullObjectException() { }
    public NullObjectException(string? message) : base(message) { }
}

internal class TypeMismatchException : DomainErrorException
{
    public TypeMismatchException() { }
    public TypeMismatchException(string? message) : base(message) { }
}

internal class RangeErrorException : DomainErrorException
{
    public RangeErrorException() { }
    public RangeErrorException(string? message) : base(message) { }
}

internal class OutOfRangeException : RangeErrorException
{
    public OutOfRangeException() { }
    public OutOfRangeException(string? message) : base(message) { }
}

internal class NegativeValueException : RangeErrorException
{
    public NegativeValueException() { }
    public NegativeValueException(string? message) : base(message) { }
}

internal class NullValueException : RangeErrorException
{
    public NullValueException() { }
    public NullValueException(string? message) : base(message) { }
}

internal class NumericErrorException : FailureException
{
    public NumericErrorException() { }
    public NumericErrorException(string? message) : base(message) { }
}

internal class ZeroDivisorException : NumericErrorException
{
    public ZeroDivisorException() { }
    public ZeroDivisorException(string? message) : base(message) { }
}

internal class NumericOverflowException : NumericErrorException
{
    public NumericOverflowException() { }
    public NumericOverflowException(string? message) : base(message) { }
}

internal class NumericUnderflowException : NumericErrorException
{
    public NumericUnderflowException() { }
    public NumericUnderflowException(string? message) : base(message) { }
}

internal class ProgramErrorException : FailureException
{
    public ProgramErrorException() { }
    public ProgramErrorException(string? message) : base(message) { }
}

internal class MethodNotImplementedException : ProgramErrorException
{
    public MethodNotImplementedException() { }
    public MethodNotImplementedException(string? message) : base(message) { }
}

internal class AbortiveTransactionException : FailureException
{
    public AbortiveTransactionException() { }
    public AbortiveTransactionException(string? message) : base(message) { }
}

internal class NotDoneException : FailureException
{
    public NotDoneException() { }
    public NotDoneException(string? message) : base(message) { }
}

internal class InfiniteSolutionsException : FailureException
{
    public InfiniteSolutionsException() { }
    public InfiniteSolutionsException(string? message) : base(message) { }
}

internal class UndefinedException : FailureException
{
    public UndefinedException() { }
    public UndefinedException(string? message) : base(message) { }
}

internal class UndefinedDerivativeException : DomainErrorException
{
    public UndefinedDerivativeException() { }
    public UndefinedDerivativeException(string? message) : base(message) { }
}

internal class UndefinedValueException : DomainErrorException
{
    public UndefinedValueException() { }
    public UndefinedValueException(string? message) : base(message) { }
}

internal class NullVectorException : DomainErrorException
{
    public NullVectorException() { }
    public NullVectorException(string? message) : base(message) { }
}
