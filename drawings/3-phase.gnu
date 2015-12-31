# use: gnuplot 3-phase.gnu
#
# uncomment for generate image
set terminal svg size 600,400
set output '3-phase.svg'

set angles degrees
set xrange [-360:360]
set style line 1 lt 1 lw 2 linecolor rgb "red"
set style line 2 lt 1 lw 1 linecolor rgb "green"
set style line 3 lt 1 lw 1 linecolor rgb "blue"

set xtics 30
set ytics 0.1

plot sin(x) ls 1, sin(x+120) ls 2, sin(x+240) ls 3

# uncomment if need wait for keypress
#pause -1
