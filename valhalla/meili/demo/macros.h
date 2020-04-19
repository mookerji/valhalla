#ifndef MMP_DEMO_MACROS_H_
#define MMP_DEMO_MACROS_H_

// Macros

// TODO(mookerji): Separate into DISALLOW_COPY, DISALLOW_ASSIGN, DISALLOW_MOVE, etc.
#define VL_DISALLOW_COPY_AND_ASSIGN(TypeName)                                                        \
  TypeName(const TypeName&) = delete;                                                                \
  TypeName(TypeName&&) = delete;                                                                     \
  TypeName& operator=(const TypeName&) = delete;                                                     \
  TypeName& operator=(TypeName&&) = delete;

#endif // MMP_DEMO_MACROS_H_
