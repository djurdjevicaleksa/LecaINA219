# LecaINA219 library for using TI INA219 current and voltage sensor

# Compilation
The library is built by running the `build.sh` script. It is built as a shared library, and therefore needs to be linked against your program using `liblecaina219.so` found inside `/build` folder after compiling. To use it, specify path to `liblecaina219.so` during your program compilation.

# Example
I provided an example program `test.c` to test the library.

# Known errors
Currently, shunt voltage reading is inaccurate.