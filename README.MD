**15-puzzle problem**


**Overview:**
This program simulates the three algorithms: 
- BFS
- DFS
- Informed Search using Heuristics



**Compilation**
To compile the C++ source code, navigate to the source directory in your terminal and use a C++ compiler 
- Compile for use with **Linux** via g++:  
*** ->     g++ 15problem.cpp -o 15problem                                                                 ***

- Compile for use with **Windows (compiled within WSL)** via mingw-w64: 
*** ->     x86_64-w64-mingw32-g++ 15problem.cpp -o 15problem.exe -static-libgcc -static-libstdc++         ***

*In both cases, instead of navigating to the source folder, one can also specify the absolute or relative path to where the source code is located in order to compile it*



**Starting the Program:**
- **Linux**: navigate to the source directory in your terminal and execute:  
*** ->     ./15problem                                                                                   ***

- **Windows**: navigate to the source directory in your terminal and double click the executable or execute 
*** ->     .\15problem.exe                                                                               ***

*In both cases, instead of navigating to the source folder, one can also specify the absolute or relative path to where the executable is located in order to run it*



**Running the Program** 
- Upon start, the program will let the user choose between using the **default puzzle layout** (the one provided in the assignment Word document) with **1** or a **custom layout** with **2**
- The user can also enter **q** to **quit** the program
- If a custom layout is chosen, the program will instruct the user to enter the numbers 0-15 in the desired order, separated by spaces, where 0 represents a blank tile.
- The program will then prompt the user to select one of the following algorithms: **1** for **BFS**, **2** for **DFS**, or **3** for **Informed Search using Heuristics**
- If '3' is chosen, the user is then prompted to choose: **1** for **h1(x) - number of misplaced tiles**, **2** for **h2(x) - sum of the distances of every tile to its goal positions**,
or **3** for **h3(x) - sum of the distances of every tile to its goal positions (weighted)**
