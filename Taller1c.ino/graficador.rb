#!/usr/bin/ruby
require 'rubygems'
require 'serialport'

if (ARGV.first.nil?) then
  max_lines = 100
else
  max_lines = ARGV.first.to_i
end

puts "Un momento por favor... "

setpoint = nil
p,i,d = 0,0,0
lines = 0
output = File.open('/tmp/salida','w')
setpoint_output = File.open('/tmp/setpoint', 'w')

begin
  s = SerialPort.new("/dev/ttyACM0", 115200, 8, 1, SerialPort::NONE)
rescue Errno::ENOENT
  s = SerialPort.new("/dev/ttyUSB0", 115200, 8, 1, SerialPort::NONE)
end
s.read_timeout = 5000

#puts "Reseteando Arduino..."
#s.dtr = 0
#s.flush_input

begin
	while (l = s.gets)
	  if (l =~ /setpoint: (.+) p: (.+) i: (.+) d: (.+)/)
		puts "Captura iniciada..."
		setpoint = $1.to_f
		p = $2.to_i
		i = $3.to_i
		d = $4.to_i
	  elsif (!setpoint.nil?)
		if (lines >= max_lines)
		  break
		else
		  output << l
		  lines += 1
		end
	  end 
	end
rescue ArgumentError
  puts "Ups. Necesito que me vuelvas a correr..."
  exit(1)
end

s.close

output.close
setpoint_output.close
puts "Captura finalizada."

puts "Graficando..."

system("setpoint=#{setpoint} gnuplot -p 'grafico.plot'")
system("setpoint=#{setpoint} gnuplot 'grafico-archivo.plot'")

File.rename('grafico.png', "grafico-p_#{p}_i_#{i}_d_#{d}.png")
