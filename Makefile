clean:
ifeq ($(OS), Windows_NT)
	echo "You are on windows"
else
	echo "Implement this on linux :-)"
endif

test:	
	python -m phand_ros_api test

run:	
	python -m phand_ros_api run

docu-html:
	$(MAKE) -C './docs' html

docu-clean:
	$(MAKE) -C './docs' clean