CLICK_CONFIGURATIONS=jaldimac-master
CLICK_CONFIGURATION_FILES=$(addprefix $(OUTPUT)/,$(addsuffix .click,$(CLICK_CONFIGURATIONS)))
CLICK_CONFIGURATION_DOT=$(addprefix $(OUTPUT)/,$(addsuffix .dot,$(CLICK_CONFIGURATIONS)))
CLICK_CONFIGURATION_GRAPH=$(addprefix $(OUTPUT)/,$(addsuffix .pdf,$(CLICK_CONFIGURATIONS)))
OUTPUT=build

.PHONY: all clean configurations pretty
.SILENT:

all: configurations pretty

%.click: ../configs/%.slick
	gcc -E -x c $< -o $@.tmp
	#click -q $@ || (rm -f $@ && false)
	click -q $@.tmp && mv $@.tmp $@

%.dot: %.click
	click-pretty --dot $< > $@

%.pdf: %.dot
	dot -Tpdf -o$@ $<

configurations: $(CLICK_CONFIGURATION_FILES)

pretty: $(CLICK_CONFIGURATION_DOT) $(CLICK_CONFIGURATION_GRAPH)
	
clean:
	cd $(OUTPUT); rm -f *
