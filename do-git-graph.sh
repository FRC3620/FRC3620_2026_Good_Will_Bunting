#!/bin/bash

git-graph --debug -m -g 4x8 2> gitgraph.log > gitgraph.gv
dot -T pdf -o gitgraph-all.pdf < gitgraph.gv
dot -T png -Gsize=34,88\! -Gdpi=300 -o gitgraph-all.png < gitgraph.gv
