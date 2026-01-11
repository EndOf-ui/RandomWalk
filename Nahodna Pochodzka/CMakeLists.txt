cmake_minimum_required(VERSION 3.25)
project(RandomWalkPOS C)

set(CMAKE_C_STANDARD 11)

add_executable(server
        server_main.c
        sim.c
        sim.h
        socket.c
        socket.h)

add_executable(client
        client_main.c
        socket.c
        socket.h)
