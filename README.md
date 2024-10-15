# Open Street Map - CS 251 (Data Structures)

**Semester:** Fall 2023  
**Instructor:** Professor Drishika Dey  

## Description

CampusMap is a console application designed to assist users in navigating the University of Illinois at Chicago (UIC) campus. The application leverages Dijkstra’s algorithm to determine the shortest weighted path between any two locations on campus. This project utilizes OpenStreetMap data for accurate navigation and TinyXML for parsing XML files.

## Features

- **Shortest Path Calculation:** Uses Dijkstra’s algorithm to find the shortest path.
- **OpenStreetMap Integration:** Parses map data from OpenStreetMap using TinyXML.
- **Console Interface:** Easy-to-use command-line interface for input and output.

## Getting Started

### Prerequisites

- Git
- C++11 compatible compiler
- Make (for Linux and Mac)

### Clone the Repository
```
git clone https://github.com/sharmaaaryan4012/Open-Street-Maps-CS251_P5.git
```


### Compile the Project
#### Linux and Mac
```
make build
```

#### Windows
```
g++ -O2 -std=c++11 -Wall main.cpp dist.cpp osm.cpp tinyxml2.cpp -o program.exe
```


### Run the Program
#### Linux and Mac
```
make run
```

#### Windows
```
.\program.exe
```

