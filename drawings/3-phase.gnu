# use: gnuplot 3-phase.gnu
#
# uncomment for generate image and comment "pause -1"
set terminal svg size 1024,768
set output '3-phase.svg'

set angles degrees
set xrange [-360:360]
set style line 1 lt 1 lw 2 linecolor rgb "red"
set style line 2 lt 1 lw 1 linecolor rgb "green"
set style line 3 lt 1 lw 1 linecolor rgb "blue"

set style line 102 lc rgb '#d6d7d9' lt 0 lw 1
set grid back ls 102

set xtics 30

set ytics 0.1

set x2tics 0.5
# 3.14 / 180 = 0.017453292
set link x2 via 0.017453292*x inverse x/0.017453292

set arrow from 230,1.0 to 230,-1.0 nohead lc rgb "orange" lw 4

plot sin(x) ls 1, sin(x+120) ls 2, sin(x+240) ls 3



# uncomment if need wait for keypress and comment two first string with svg
#pause -1
