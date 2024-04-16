gcc -fPIC -shared src/lecaina219.c -o build/liblecaina219.so
gcc test.c -o test build/liblecaina219.so -I./src -lm