# Open Street Map

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
cd Open-Street-Maps-CS251_P5
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
```
make run
```

### Navigating UIC open street map

1. **Enter map filename:**
    ```
    select map.osm
    ```

2. **Navigating UIC open street map:**
    ```
    Enter map filename> 
    map.osm 
    # of nodes: 18297
    # of footways: 382
    # of buildings: 34

    # of vertices: 18297
    # of edges: 3596
    ```

3. **Enter person 1's building (partial name or abbreviation), or #:**
4. **Enter person 2's building (partial name or abbreviation):**

Choose from the list of buildings and use the building code (it is case sensitive).


