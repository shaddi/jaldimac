# ========================================
# Configuration
# ========================================

CONFIGURATIONS=master station-1 station-2 station-3 station-4
ELEMENTS_CONFIGURATION=--enable-userlevel

# ========================================
# Internal and derived variables
# ========================================

CONFIGDIR=configs
ELEMENTDIR=elements
BUILDDIR=build
DISTDIR=dist
CONFIGURATION_FILES=$(addprefix $(BUILDDIR)/,$(addsuffix .click,$(CONFIGURATIONS)))
CONFIGURATION_DOT=$(addprefix $(BUILDDIR)/,$(addsuffix .dot,$(CONFIGURATIONS)))
CONFIGURATION_GRAPH=$(addprefix $(DISTDIR)/,$(addsuffix .pdf,$(CONFIGURATIONS)))
ARCHIVES=$(addprefix $(DISTDIR)/,$(addsuffix .clickpkg,$(CONFIGURATIONS)))
JALDI_PACKAGE_BINARY=$(ELEMENTDIR)/jaldi.uo

# Metarules
.PHONY: all clean configurations pretty elements
.SILENT:

# Rules
all: dist

%.click: ../$(CONFIGDIR)/%.slick
	gcc -E -x c $< -o $@

%.dot: %.click
	click-pretty --dot $< > $@

%.pdf: %.dot
	dot -Tpdf -o$@ $<

%.clickpkg: ../$(BUILDDIR)/%.click configurations elements
	cp $< $(BUILDDIR)/config
	ar rcs $@ $(BUILDDIR)/config $(ELEMENTDIR)/jaldi.uo
	click -q $@ || (rm -f $@ && false)

configurations: $(CONFIGURATION_FILES)

pretty: $(CONFIGURATION_DOT) $(CONFIGURATION_GRAPH)

$(ELEMENTDIR)/Makefile:
	cd $(ELEMENTDIR) && autoconf && ./configure $(ELEMENTS_CONFIGURATION)

$(JALDI_PACKAGE_BINARY): $(ELEMENTDIR)/Makefile
	cd $(ELEMENTDIR) && make

elements: $(JALDI_PACKAGE_BINARY)

dist: $(ARCHIVES) pretty

clean:
	cd $(BUILDDIR); rm -f *
	cd $(DISTDIR); rm -f *
	cd $(ELEMENTDIR) && make clean
	cd $(ELEMENTDIR) && rm Makefile configure config.status config.log
	cd $(ELEMENTDIR) && rm -rf autom4te.cache
