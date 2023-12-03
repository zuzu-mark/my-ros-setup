all:
	clang++ -S -emit-llvm main.cpp -o - | \
		opt -enable-new-pm=0  -analyze -dot-callgraph --callgraph-dot-filename-prefix=test
b:
	dot -Tpng test.callgraph.dot > test.png
c:
	cat test.callgraph.dot | \
	c++filt | \
	sed 's,>,\\>,g; s,-\\>,->,g; s,<,\\<,g' | \
	gawk '/external node/{id=$$1} $$1 != id' | \
	dot -Tpng -ocallgraph.png
