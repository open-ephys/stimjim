EXN-23359.DXF is the original outline of the extruded aluminum enclosure. It was
edited in panel-outline.SLDPRT to remove some of the finer features of the
aluminum enclosure that might not be good PCB DFM. I extruded the sketch in
panel-outline.SLDPRT because exporting the sketch by itself was presenting buggy
behavior wherein not all lines of the sketch were actually exported into the
resulting DXF. Instead, I export the outline of the extruded part which doesn't
present this buggy behavior. panel-outline.SLDPRT is exported to
panel-outline.DXF which is subsequently imported into the KiCAD PCB panel files
as a graphic.