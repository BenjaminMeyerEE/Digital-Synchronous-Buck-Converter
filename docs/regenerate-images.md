# Regenerating the images

Everything in `docs/` is generated from the design files with `kicad-cli`, so the
images can be rebuilt after any board change rather than re-screenshotted.

On Windows the binary is at:

    "C:/Program Files/KiCad/10.0/bin/kicad-cli.exe"

Run all commands from the repository root.

## 3D renders

Isometric view (README header):

    kicad-cli pcb render -o docs/board-iso.png --quality high --perspective \
      --rotate '-30,0,-35' --zoom 0.9 --width 2000 --height 1500 \
      --background transparent hardware/synch_buck_digital.kicad_pcb

Top-down view:

    kicad-cli pcb render -o docs/board-top.png --side top --quality high \
      --width 2000 --height 1500 --background transparent \
      hardware/synch_buck_digital.kicad_pcb

Transparent backgrounds are deliberate — they read correctly on both GitHub's
light and dark themes.

## Copper layers

One SVG per copper layer, each with the board outline for reference:

    for pair in "F.Cu:layer-f-cu" "In1.Cu:layer-in1-cu" \
                "In2.Cu:layer-in2-cu" "B.Cu:layer-b-cu"; do
      L="${pair%%:*}"; N="${pair##*:}"
      kicad-cli pcb export svg -o "docs/$N.svg" --mode-single \
        --layers "$L,Edge.Cuts" --exclude-drawing-sheet --page-size-mode 2 \
        --check-zones hardware/synch_buck_digital.kicad_pcb
    done

Then darken the board outline, because KiCad plots `Edge.Cuts` as `#D0D2CD`,
which disappears against a white page:

    sed -i 's/#D0D2CD/#555555/g' docs/layer-*.svg

Two deliberate choices:

- **Silkscreen is excluded.** KiCad plots it as `#F2EDA1`, effectively invisible
  on GitHub's light background.
- **`B.Cu` is not mirrored.** Plotting every layer from the same side means
  features line up when comparing layers, which is the point of showing them.

## Schematic

    kicad-cli sch export pdf -o docs/schematic.pdf \
      hardware/synch_buck_digital.kicad_sch

## Notes

- `--check-zones` refills copper pours before plotting, so layer images always
  reflect the current zone settings.
- Close the KiCad PCB editor before regenerating. `kicad-cli` reads from disk,
  and an open editor can overwrite the board file when it saves.
