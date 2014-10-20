#Gnuplot script for plotting data in "feat.txt"
#Invoke with gnuplot> load 'PlotFeatures.plot'
set autoscale
unset log
unset label
set term postscript eps color linewidth 1 font "Times-Roman,16" size 13cm,6.5cm
set size ratio 0.42
set xtic auto
set ytic auto
set title "Pose Recognition (with Synthetic Database)"
set xlabel "Rank"
set ylabel "Accumulated Recognition Rate (%)"
set xr [1:40]
set yr [0:100]
set grid 
show grid
set key inside r b box horizontal 
set style fill solid
set linetype 1 lt 1 lw 1 lc rgb "red" pt 5
set linetype 2 lt 1 lw 1 lc rgb "blue" pt 9
set linetype 3 lt 1 lw 1 lc rgb "green" pt 13
set linetype 4 lt 1 lw 1 lc rgb "dark-goldenrod" pt 11
set linetype 5 lt 1 lw 1 lc rgb "dark-violet" pt 7
set output 'plot.eps'
plot "feat.txt" u 1:2 t "VFH" w linespoints,"feat.txt" u 1:3 t "ESF" w linespoints,"feat.txt" u 1:4 t "CVFH" w linespoints,"feat.txt" u 1:5 t "OUR-CVFH" w linespoints,"feat.txt" u 1:6 t "COMPOSITE" w linespoints
