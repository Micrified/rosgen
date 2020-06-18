rosgen: ros_env.c ros_gen.c ros_parse.c ros_semantics.c ros_synth.c
	gcc -std=c99 -o $@ $^

install: rosgen
	sudo cp $^ /usr/local/bin/

clean: rosgen
	rm $^