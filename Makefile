# Tutoriel DEM 2D -- compilation des corrigés
#   make            compile toutes les étapes dans build/
#   make etape3     compile une seule étape
#   make test       compile et exécute toutes les étapes (sorties dans build/)
#   make clean
CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra
SRC      := corriges
ETAPES   := etape1 etape2 etape3 etape4 etape5 etape5_tests etape6 etape7 etape8

etape1_src := $(SRC)/etape1_chute_libre.cpp
etape2_src := $(SRC)/etape2_rebond.cpp
etape3_src := $(SRC)/etape3_collision.cpp
etape4_src := $(SRC)/etape4_boite.cpp
etape5_src := $(SRC)/etape5_frottement.cpp
etape5_tests_src := $(SRC)/etape5_tests.cpp
etape6_src := $(SRC)/etape6_verlet.cpp
etape7_src := $(SRC)/etape7_cellules.cpp
etape8_src := $(SRC)/etape8_effondrement.cpp

all: $(ETAPES)

$(ETAPES): %: build/%

build/%: $(SRC)/*.cpp $(SRC)/*.hpp viz/dem_viz.hpp
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Iviz -I$(SRC) $($*_src) -o $@

test: all
	@for e in $(ETAPES); do echo "=========== $$e ==========="; (cd build && ./$$e) || exit 1; done

clean:
	rm -rf build

.PHONY: all test clean $(ETAPES)
