#pragma once
/*
This provides macros to enable/disable special member functions that can be
defaulted i.e. copy-construction, copy-assignment, move-construction, and
move-assignment.

When enabled via the macros, the '= default' implementation is provided. Code
that needs custom implementation of special member functions should not use
these macros.
*/

/*
ADI_NO_COPY_NO_MOVE_NO_ASSIGN deletes the special member functions for copy,
move and assignment. Invoke this macro is the public section of the class
declaration: class Foo { public: ADI_NO_COPY_NO_MOVE_NO_ASSIGN(Foo)
        ....
        ....
};
*/

#define ADI_NO_COPY_NO_MOVE_NO_ASSIGN(Classname)                               \
  Classname(const Classname &) = delete;                                       \
  void operator=(const Classname &) = delete;                                  \
  Classname(Classname &&) = delete;                                            \
  void operator=(Classname &&) = delete;
/*
ADI_DEFAULT_COPY_MOVE_ASSIGN defaults the special member functions for copy,
move and assignment. This macro should be invoked when copy construction and
copy assignment functions are well formed. Invoke this macro is the public
section of the class declaration: class Foo { public:
        ADI_DEFAULT_COPY_MOVE_ASSIGN(Foo)
        ....
        ....
};
*/

#define ADI_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname)                        \
  Classname(const Classname &) = default;                                      \
  Classname &operator=(const Classname &) = default;                           \
  Classname(Classname &&) = default;                                           \
  Classname &operator=(Classname &&) = default;                                \
  /* Fails at compile-time if default-copy doesn't work. */                    \
  static void ADI_COPYABLE_DEMAND_COPY_CAN_COMPILE() {                         \
    (void)static_cast<Classname &(Classname::*)(const Classname &)>(           \
        &Classname::operator=);                                                \
  }