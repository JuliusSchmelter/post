# POST

This is a rewrite of the Program to Optimize Simulated Trajectories (POST) by NASA.
It was written in 1975 and since then built upon. The fundamental program is described in detail in public papers released by NASA, which were used to write this version in Rust.

This program is an adaptation of the Program to Simulate Optimized Trajectories (POST).
It is a program to optimize a 3 Degrees of Freedom (DoF) trajectory of a vehicle, e.g. a
rocket ascending through the atmosphere.

POST was written in 1970 by the Martin Marietta Corporation for NASA. The program
is written in Fortran 4 and was mainly developed to simulate the space shuttle, it was
later built upon and greatly improved. Its successor, POST2, is still in use today, for
example for the Artemis program or for Perseverance.

This project tries to use the original user manuals to rewrite the program in a more modern programming language: Rust.

Please visit the [user manual](https://tiborvoelcker.github.io/post/manual.pdf) to find out more about the program.

If you want to understand the code, visit the [code documentation](https://tiborvoelcker.github.io/post/docs/post/index.html).
