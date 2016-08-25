set term png
set output 'grafico.png'

setpoint=system("echo $setpoint") + 0

set size 2.0,2.5
set multiplot layout 1,2

set grid ytics lc rgb "#bbbbbb" lw 2 lt 0
set grid xtics lc rgb "#bbbbbb" lw 2 lt 0
set xrange [0: 1000]

set xtics 250
set xlabel 't [ms]'
set ylabel 'v [°/ms]'
plot '/tmp/salida' using 6:2 with lines title 'velocidad',\
     setpoint title 'setpoint'

set yrange [-260: 260]
set ytics 50
set xtics 250
set xlabel 't [ms]'
set ylabel 'PWM []'
plot '/tmp/salida' using 6:4 with lines title 'output'

unset multiplot


pause -1
