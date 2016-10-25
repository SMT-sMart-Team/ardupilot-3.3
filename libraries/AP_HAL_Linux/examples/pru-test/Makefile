
all:
	arm-linux-gnueabihf-g++ -std=gnu++11 --static -g RCOutput_PRU_test.cpp -o pwmtest
	arm-linux-gnueabihf-g++ -std=gnu++11 --static -g RCInput_PRU_test.cpp -o rcintest
	arm-linux-gnueabihf-strip pwmtest rcintest

clean:
	rm -f pwmtest
	rm -f rcintest
