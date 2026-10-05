* Use the `lidar` conda environment for Python commands and tests. Its GTSAM wrapper is built from `gtsam-private` and links to `/Users/dellaert/gtsam-private/lib`. Do not substitute a public GTSAM wheel or the temporary `py312` wrapper.

For reviewing PRs:
* All functions in header files should have doxygen-style API docs, /** */ style, except small functions like getters which can have single line /// comments, no need for @brief, @params etc
* Use /// for single-line comments rather than /** */
* Use meaningful variable names, e.g. `measurement` not `msm`, avoid abbreviations.
* Flag overly complex or long/functions: break up in smaller functions
* Classes are Uppercase, methods and functions lowerMixedCase.
* Public fields in structs keep plain names (no trailing underscore).
* Apart from those naming conventions, we adopt Google C++ style.
* Prefer concise, elegant examples: use the fewest helpers possible, favor direct construction and small local functors/lambdas over extra adapter functions.
* After any code change, always run relevant tests via `make -j6 testXXX.run` in the build folder $WORKSPACE/build. If in VS code, ask for escalated permissions if needed.
