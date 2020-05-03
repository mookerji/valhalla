#ifndef MMP_DEMO_MACROS_H_
#define MMP_DEMO_MACROS_H_

// Macros

#define VL_DISALLOW_COPY(TypeName) TypeName(const TypeName&) = delete

#define VL_DISALLOW_ASSIGN(TypeName) TypeName& operator=(const TypeName&) = delete

#define VL_DISALLOW_MOVE(TypeName)                                                                   \
  TypeName(TypeName&&) = delete;                                                                     \
  TypeName& operator=(TypeName&&) = delete;

#define VL_DISALLOW_COPY_AND_ASSIGN(TypeName)                                                        \
  VL_DISALLOW_COPY(TypeName);                                                                        \
  VL_DISALLOW_ASSIGN(TypeName);                                                                      \
  VL_DISALLOW_MOVE(TypeName);

#endif // MMP_DEMO_MACROS_H_
