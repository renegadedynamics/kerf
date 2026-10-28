// Common support for the generated extern "C" shim over OCCT.
//
// ABI conventions (mirrored by tools/OcctGen and Build123d.Occt):
// - Objects cross as opaque pointers. Value classes are heap copies owned by the caller and freed with
//   b3d_<Class>_delete; Standard_Transient classes are retained once per pointer handed out and
//   released with b3d_<Class>_delete.
// - bool crosses as uint8_t and enums as int32_t.
// - Every function takes a trailing b3d_error** that is null on success, and otherwise owns an error the
//   caller reads and frees with b3d_error_free.
#pragma once

#include <Standard_Failure.hxx>
#include <Standard_Transient.hxx>

#include <cstdint>
#include <exception>

#ifdef _WIN32
  #define B3D_API extern "C" __declspec(dllexport)
#else
  #define B3D_API extern "C" __attribute__((visibility("default")))
#endif

struct b3d_error;

b3d_error* b3d_make_error(const char* type, const char* message) noexcept;

#define B3D_CATCH(err)                                                                             \
  catch (const Standard_Failure& e)                                                                \
  {                                                                                                \
    *(err) = b3d_make_error(e.ExceptionType(), e.what());                                          \
  }                                                                                                \
  catch (const std::exception& e)                                                                  \
  {                                                                                                \
    *(err) = b3d_make_error("std::exception", e.what());                                           \
  }                                                                                                \
  catch (...)                                                                                      \
  {                                                                                                \
    *(err) = b3d_make_error("unknown", "unknown native exception");                                \
  }

//! Takes one reference on a transient for the caller; null stays null.
template <class T>
T* b3d_retain(T* p) noexcept
{
  if (p != nullptr)
    p->IncrementRefCounter();
  return p;
}

//! Drops a reference taken by b3d_retain, deleting the object with its last reference.
inline void b3d_release(Standard_Transient* p) noexcept
{
  if (p != nullptr && p->DecrementRefCounter() == 0)
    p->Delete();
}
