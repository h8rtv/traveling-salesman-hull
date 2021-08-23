GCC = g++

all:
	$(GCC) src/tsp.cpp -o tsp

clean:
	rm -rf tsp *.txt *.pdf

genpoints:
	scripts/genpoints 3

view:
	scripts/visualizar.sh

run:
	./tsp input.txt

default: all
