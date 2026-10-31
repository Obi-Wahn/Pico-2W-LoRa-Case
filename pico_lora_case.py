#!/usr/bin/env python3
"""
Gehäuse für Raspberry Pi Pico 2 W + Waveshare Pico-LoRa-SX1262 + 600-mAh-LiPo
=============================================================================

Parametrisch, erzeugt zwei STL-Dateien:
  pico_lora_box.stl  – Unterteil (aufrecht drucken, ohne Support)
  pico_lora_lid.stl  – Deckel (mit der flachen Seite aufs Bett, ohne Support)

Benötigt:  pip install manifold3d trimesh numpy
Aufruf:    python3 pico_lora_case.py [ausgabeordner]   (Standard: ./stl neben dem Skript)
           Maße unten anpassen und neu ausführen.

Koordinaten (Innenraum): X = Länge (USB-Ende bei X=0, Antennen-Ende hinten),
Y = Breite (Platinen-Stapel mit Pin-1-Rand an der Wand bei Y=0, Akku auf der Pin-40-Seite),
Z = Höhe ab Innenboden.
Alle Maße in mm.
"""
import numpy as np
import trimesh
from manifold3d import Manifold, CrossSection, JoinType, OpType

# ---------------------------------------------------------------------------
# PARAMETER – hier anpassen
# ---------------------------------------------------------------------------
WALL      = 2.0    # Wandstärke
FLOOR     = 2.0    # Bodenstärke
LID_T     = 2.0    # Deckelstärke
R_OUT     = 3.0    # Eckenradius außen
FIT       = 0.2    # Spiel Deckel-Lippe zu Wand

# --- Platinen-Stapel (aus deinen Fotos gemessen, ggf. nachmessen) ---
PIN_BELOW     = 6.2   # wie weit die Header-Pins unten aus dem LoRa-Board ragen
UNDER_CLR     = 1.3   # Luft zwischen Pin-Spitzen und Boden
LORA_PCB      = 1.6
FEMALE_HDR    = 9.0   # Höhe Buchsenleiste auf dem LoRa-Board
MALE_PLASTIC  = 2.6   # Kunststoffsteg der Stiftleiste unter dem Pico
PICO_PCB      = 1.0
PICO_COMP_H   = 4.5   # höchstes Bauteil auf dem Pico (Debug-Stecker)
TOP_CLR       = 1.0   # Luft über den Bauteilen

BOARD_W       = 21.0
PICO_L        = 51.0
LORA_L        = 52.0
USB_OVERHANG  = 1.1   # Micro-USB-Buchse steht so weit über die Pico-Platine
USB_GAP       = 0.2   # Luft zwischen USB-Buchse und Innenwand
LORA_OFFSET   = 0.6   # LoRa-Platine beginnt so weit hinter der USB-Buchsenfront
SIDE_CLR      = 0.3   # seitliche Luft um die Platinen

# --- USB-Öffnung (großzügig für dicke Stecker) ---
USB_OPEN_W    = 13.5
USB_OPEN_H    = 10.5
USB_POCKET_W  = 19.0  # Außentasche: Wand dort nur noch USB_POCKET_REST dick
USB_POCKET_H  = 15.0
USB_POCKET_REST = 1.0

# --- Akku: Waveshare 851451 (8,5 x 14 x 51 mm, gemessen inkl. Falz) ---
BAT_L = 52.5          # Länge inkl. gefalteter Kopfseite
BAT_W = 14.5          # Breite inkl. Seitenfalze
BAT_T = 8.8           # Dicke
BAT_ON_EDGE = True    # hochkant stellen -> schmaleres Gehäuse
BAT_CLR = 1.0         # Spiel im Akkufach

# Lage im Fach: Grundfläche (quer) x Höhe
BAT_FOOT = BAT_T if BAT_ON_EDGE else BAT_W
BAT_H    = BAT_W if BAT_ON_EDGE else BAT_T   # Niederhalter im Deckel richten sich danach

# --- Antenne ---
SMA_HOLE_D   = 6.6    # SMA-Durchführung (1/4"-36-Gewinde)
SMA_RIGID    = 12.5   # steifer Teil innen: Mutter + Buchsenkörper + Crimphülse (an deinen Fotos gemessen)
SMA_NUT_L    = 5.2    # Mutter + Buchsenkörper innen (Ø 9,2 über Ecken)
SMA_ABOVE_BAT = 3.0   # Luft zwischen Crimphülse und Akku-Oberkante
ANT_SPACE    = 15.0   # Platz zwischen Platinen-Ende und Antennenwand

# Kabelspule im Deckel: das lange IPEX-SMA-Kabel wird um einen ovalen Kern gewickelt
CABLE_SPOOL  = True
SPOOL_H      = 3.4    # Kernhöhe unter dem Deckel
SPOOL_WALL   = 1.2    # Wandstärke des Kerns
SPOOL_CH     = 2.4    # Kanalbreite zwischen Kern und Deckel-Lippe
SPOOL_TAB    = 1.3    # Haltelaschen ragen so weit nach außen
SPOOL_TAB_W  = 4.0    # Laschenbreite

# --- Ein/Aus-Schalter (gemessen an deinem Foto der LoRa-Unterseite) ---
# Sitzt unter dem LoRa-Board direkt an der USB-Kante. Der Schieber ragt ~0,5 mm über
# die Platinenkante Richtung USB-Ende und läuft quer: OFF = Pin-40-Seite (Akku),
# ON = Pin-1-Seite (Außenwand). Von vorn auf die USB-Seite gesehen: links AUS, rechts AN.
SWITCH_SLOT     = True
SW_SLIDER_OFF   = 8.2   # Schiebermitte in OFF, ab Pin-1-Rand
SW_SLIDER_W     = 1.6
SW_TRAVEL       = 2.5   # Schiebeweg (großzügig angenommen)
SW_BODY_Y0, SW_BODY_Y1 = 2.5, 11.9   # Schaltergehäuse quer, ab Pin-1-Rand
SW_BODY_X0, SW_BODY_X1 = 1.45, 4.4   # Schaltergehäuse längs, ab LoRa-Kante
SW_BODY_H       = 1.8   # Höhe unter der Platine (geschätzt)
SW_SLOT_MARGIN  = 0.8   # Luft im Schlitz links/rechts
SW_SLOT_DOWN    = 2.6   # Schlitz reicht so weit unter die LoRa-Platine ...
SW_SLOT_UP      = 0.15  # ... und endet so knapp darunter
SW_SLOT_REST    = 1.0   # Wand um den Schlitz außen auf 1 mm ausgedünnt (Fingernagel)

# --- Akkubuchse PH1.25 (Unterseite; Stecker zeigt zur Pin-40-Seite -> Akku liegt dort) ---
CONN_X0, CONN_X1 = 6.9, 14.7   # Buchse längs, ab LoRa-Kante
CONN_Y0, CONN_Y1 = 4.4, 7.7    # Buchse quer, ab Pin-1-Rand
PLUG_Y1          = 10.4        # Stecker reicht bis hier, Kabel läuft weiter zur Pin-40-Seite
CABLE_X0, CABLE_X1 = 9.9, 11.6 # Kabel geht zwischen 4. und 5. Pin (ab USB-Ende) der Pin-40-Reihe durch

LED_SLOT      = True  # Lichtschlitz für PWR/CHG-LEDs (Pin-1-Seite, ca. 4 mm vom Rand)
LED_Y         = 4.0
SNAPS         = True  # kleine Rastnasen am Deckel

SEG = 64

# ---------------------------------------------------------------------------
# Abgeleitete Maße
# ---------------------------------------------------------------------------
LORA_Z   = PIN_BELOW + UNDER_CLR                  # Unterkante LoRa-Platine
PICO_Z   = LORA_Z + LORA_PCB + FEMALE_HDR + MALE_PLASTIC  # Unterkante Pico
PICO_TOP = PICO_Z + PICO_PCB
USB_CZ   = PICO_TOP + 1.4                          # Mitte Micro-USB-Buchse

USB_FACE_X = USB_GAP
PICO_X0    = USB_FACE_X + USB_OVERHANG
LORA_X0    = USB_FACE_X + LORA_OFFSET
LORA_X1    = LORA_X0 + LORA_L
# Y-Aufteilung: Platinen-Stapel (Pin-1-Rand an der Wand) | Trennsteg | Akkufach (Pin-40-Seite)
BOARD_Y0 = SIDE_CLR                          # Pin-1-Rand der Platinen
BOARD_YC = BOARD_Y0 + BOARD_W / 2
DIV_Y0   = BOARD_Y0 + BOARD_W + SIDE_CLR + 0.2
DIV_T    = 1.2
BAY_Y0   = DIV_Y0 + DIV_T
BAY_W    = BAT_FOOT + BAT_CLR
IN_W     = BAY_Y0 + BAY_W
IN_L     = max(LORA_X1, PICO_X0 + PICO_L) + ANT_SPACE
IN_H     = max(PICO_TOP + PICO_COMP_H + TOP_CLR, USB_CZ + USB_OPEN_H / 2 + 1.0)

OUT_L = IN_L + 2 * WALL
OUT_W = IN_W + 2 * WALL
BOX_H = FLOOR + IN_H

BAY_X0 = 6.0                        # vorn Platz für Kabelabgang + Kabelreserve
BAY_X1 = BAY_X0 + BAT_L + 0.8

# Pico-Befestigungslöcher (Ø 2,1 mm, 2 mm vom Ende, 4,8 mm vom Rand)
PICO_HOLES = [(PICO_X0 + 2.0, BOARD_Y0 + 4.8), (PICO_X0 + 2.0, BOARD_Y0 + 16.2),
              (PICO_X0 + 49.0, BOARD_Y0 + 4.8), (PICO_X0 + 49.0, BOARD_Y0 + 16.2)]

# SMA-Buchse seitlich versetzt über dem Akku-Ende (nicht gegenüber der IPEX-Buchse):
# das steife Stück läuft über dem Akku am Platinen-Stapel vorbei, das Kabel steigt von dort
# direkt in den Kanal der Kabelspule und kommt am Antennen-Ende zur IPEX-Buchse herunter
SMA_Y = min(BAY_Y0 + BAY_W / 2, IN_W - 4.6 - 1.2)   # Mutter frei von der Innenecke
SMA_Z = BAT_H + 1.8 + SMA_ABOVE_BAT
assert IN_L - SMA_NUT_L - 0.5 >= BAY_X1 + 1.2, "ANT_SPACE zu klein: SMA-Mutter stößt an den Akku-Anschlag"
assert SMA_Z + 4.6 <= IN_H - 3.0 - 0.5, "SMA zu hoch: Mutter stößt an die Deckel-Lippe" 

# Kabelspule: ovaler Kern zwischen den Pico-Zapfen, Kanal entlang der Deckel-Lippe
LIP_IN      = FIT + 1.4                       # Innenkante der Deckel-Lippe
SPOOL_Y0    = LIP_IN + SPOOL_CH
SPOOL_Y1    = IN_W - LIP_IN - SPOOL_CH
SPOOL_X0    = PICO_HOLES[0][0] + 1.6 + 1.2 + SPOOL_CH
SPOOL_X1    = PICO_HOLES[2][0] - 1.6 - 1.2 - SPOOL_CH
SPOOL_R     = (SPOOL_Y1 - SPOOL_Y0) / 2


# ---------------------------------------------------------------------------
# Helfer
# ---------------------------------------------------------------------------
def box(x0, y0, z0, x1, y1, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def rrect(x0, y0, x1, y1, r):
    sq = CrossSection.square([x1 - x0 - 2 * r, y1 - y0 - 2 * r]).translate([x0 + r, y0 + r])
    return sq.offset(r, JoinType.Round, circular_segments=SEG) if r > 0 else sq


def rbox(x0, y0, z0, x1, y1, z1, r):
    return rrect(x0, y0, x1, y1, r).extrude(z1 - z0).translate([0, 0, z0])


def cyl(x, y, z0, h, d, d2=None, axis="z"):
    c = Manifold.cylinder(h, d / 2, (d2 if d2 is not None else d) / 2, SEG)
    if axis == "x":
        c = c.rotate([0, 90, 0])
    elif axis == "y":
        c = c.rotate([-90, 0, 0])
    return c.translate([x, y, z0])


def sphere(x, y, z, r):
    return Manifold.sphere(r, 32).translate([x, y, z])


# ---------------------------------------------------------------------------
# Unterteil
# ---------------------------------------------------------------------------
def make_box():
    # Innenraum-Ursprung liegt bei (0,0,0); Außenhülle entsprechend versetzt
    shell = rbox(-WALL, -WALL, -FLOOR, IN_L + WALL, IN_W + WALL, IN_H, R_OUT)
    cavity = rbox(0, 0, 0, IN_L, IN_W, IN_H + 1, max(R_OUT - WALL, 0.5))
    b = shell - cavity

    # Auflage-Stege unter den Stirnkanten der LoRa-Platine (zwischen den Pin-Reihen)
    ly0, ly1 = BOARD_Y0 + 2.8, BOARD_Y0 + BOARD_W - 2.8
    # USB-Ende: hier sitzen Schalter und Akkubuchse -> nur zwei schmale Auflagen
    b += box(0, 0, 0, LORA_X0 + 5.5, BOARD_Y0 + 1.0, LORA_Z)                  # Pin-1-Rand
    b += box(0, BOARD_Y0 + SW_BODY_Y1 + 0.5, 0, LORA_X0 + 5.5,
             BOARD_Y0 + BOARD_W - 6.3, LORA_Z)                                  # neben dem Schalter
    # Antennen-Ende: durchgehende Auflage zwischen den Pin-Reihen
    b += box(LORA_X1 - 1.4, ly0, 0, LORA_X1 + 0.6, ly1, LORA_Z)
    # Anschlag hinter der Platine (verhindert Verrutschen)
    b += box(LORA_X1 + 0.6, 0, 0, LORA_X1 + 2.0, BOARD_Y0 + 2.0, LORA_Z + LORA_PCB + 3)

    # Trennsteg Platine | Akku – mit Lücke für das Akkukabel
    div_h = min(BAT_H - 1.0, LORA_Z + LORA_PCB + 4.0) if BAT_ON_EDGE else 12.0
    # vorn offen: das Akkukabel läuft vom Akku-Kopf quer zur Platine und an deren Rand
    # entlang bis zur Lücke zwischen den Pins; der Steg beginnt erst dahinter
    div_x0 = LORA_X0 + CABLE_X1 + 4.0
    b += box(div_x0, DIV_Y0, 0, BAY_X1, DIV_Y0 + DIV_T, div_h)
    # Akku-Anschläge vorn (niedrig, Kabel läuft darüber) / hinten
    stop_w = min(3.0, BAY_W - 2.0)
    yc = BAY_Y0 + BAY_W / 2
    b += box(BAY_X0 - 1.2, yc - stop_w / 2, 0, BAY_X0, yc + stop_w / 2, 3.0)
    b += box(BAY_X1, BAY_Y0 + 0.8, 0, BAY_X1 + 1.2, BAY_Y0 + BAY_W - 0.8, 6.0)

    # USB-Öffnung + Außentasche (dünne Wand rund um die Buchse)
    usb = rrect(BOARD_YC - USB_OPEN_W / 2, USB_CZ - USB_OPEN_H / 2,
                BOARD_YC + USB_OPEN_W / 2, USB_CZ + USB_OPEN_H / 2, 1.5)
    usb3d = usb.extrude(WALL + 2).rotate([90, 0, 90]).translate([-WALL - 1, 0, 0])
    b -= usb3d
    pocket = rrect(BOARD_YC - USB_POCKET_W / 2, USB_CZ - USB_POCKET_H / 2,
                   BOARD_YC + USB_POCKET_W / 2, USB_CZ + USB_POCKET_H / 2, 2.5)
    pocket_depth = WALL - USB_POCKET_REST
    b -= pocket.extrude(pocket_depth + 1).rotate([90, 0, 90]).translate([-WALL - 1, 0, 0])

    # LED-Lichtschlitz in der Stirnwand (Höhe der LoRa-Oberseite)
    if LED_SLOT:
        b -= box(-WALL - 1, BOARD_Y0 + LED_Y - 3.5, LORA_Z + LORA_PCB + 0.3, 0.5,
                 BOARD_Y0 + LED_Y + 3.5, LORA_Z + LORA_PCB + 2.8)

    # SMA-Durchführung im Antennen-Ende
    b -= cyl(IN_L - 1, SMA_Y, SMA_Z, WALL + 2, SMA_HOLE_D, axis="x")

    # Schlitz für den Schiebeschalter in der Stirnwand (Höhe Schieber, unter der LoRa-Platine)
    if SWITCH_SLOT:
        y0 = BOARD_Y0 + SW_SLIDER_OFF - SW_TRAVEL - SW_SLIDER_W / 2 - SW_SLOT_MARGIN
        y1 = BOARD_Y0 + SW_SLIDER_OFF + SW_SLIDER_W / 2 + SW_SLOT_MARGIN
        z0, z1 = LORA_Z - SW_SLOT_DOWN, LORA_Z - SW_SLOT_UP
        b -= box(-WALL - 1, y0, z0, 0.5, y1, z1)
        # Außen trichterförmig ausgedünnt, damit der Fingernagel an den Schieber kommt
        d = WALL - SW_SLOT_REST
        inner = box(-WALL + d - 0.01, y0, z0, -WALL + d, y1, z1)
        outer = box(-WALL - 0.5, y0 - d - 0.6, z0 - d - 0.6, -WALL - 0.49, y1 + d + 0.6, z1 + 0.3)
        b -= Manifold.batch_hull([inner, outer])

    # Rast-Mulden für die Deckel-Nasen
    if SNAPS:
        zc = IN_H - 1.5
        for y in (0.0, IN_W):
            b -= sphere(IN_L / 2, y, zc, 0.85)

    return b


# ---------------------------------------------------------------------------
# Deckel (modelliert in Einbaulage, wird zum Export umgedreht)
# ---------------------------------------------------------------------------
def make_lid():
    z0 = IN_H
    lid = rbox(-WALL, -WALL, z0, IN_L + WALL, IN_W + WALL, z0 + LID_T, R_OUT)

    # Lippe, die in das Unterteil greift
    lip_h, lip_t = 3.0, 1.4
    r_in = max(R_OUT - WALL - FIT, 0.5)
    outer = rbox(FIT, FIT, z0 - lip_h, IN_L - FIT, IN_W - FIT, z0 + 0.01, r_in)
    inner = rbox(FIT + lip_t, FIT + lip_t, z0 - lip_h - 1, IN_L - FIT - lip_t, IN_W - FIT - lip_t, z0 + 1,
                 max(r_in - lip_t, 0.3))
    lip = outer - inner
    # Lippe über der USB-Öffnung aussparen
    lip -= box(-1, BOARD_YC - USB_POCKET_W / 2, z0 - lip_h - 1, 6.5, BOARD_YC + USB_POCKET_W / 2, z0 + 0.5)
    lid += lip

    # Rastnasen
    if SNAPS:
        zc = IN_H - 1.5
        nub_r = 0.65
        lid += sphere(IN_L / 2, FIT + nub_r * 0.15, zc, nub_r)
        lid += sphere(IN_L / 2, IN_W - FIT - nub_r * 0.15, zc, nub_r)

    # Zapfen in die Pico-Befestigungslöcher (fixieren den ganzen Stapel)
    shoulder_bottom = PICO_TOP + 0.3
    for (x, y) in PICO_HOLES:
        lid += cyl(x, y, shoulder_bottom, z0 - shoulder_bottom + 0.01, 3.2)
        lid += cyl(x, y, shoulder_bottom - 1.6, 1.6, 1.1, 1.75)  # konische Spitze

    # Akku-Niederhalter: Längssteg über der Akkumitte
    hold_bottom = BAT_H + 0.6
    yc = BAY_Y0 + BAY_W / 2
    if CABLE_SPOOL:   # nur im geraden Teil der Spule, damit der Kabelkanal frei bleibt
        hx0, hx1 = SPOOL_X0 + SPOOL_R + 0.5, SPOOL_X1 - SPOOL_R - 0.5
    else:
        hx0, hx1 = BAY_X0 + BAT_L * 0.25, BAY_X0 + BAT_L * 0.75
    lid += box(hx0, yc - 0.8, hold_bottom, hx1, yc + 0.8, z0 + 0.01)

    if CABLE_SPOOL:
        lid += make_spool(z0)

    return lid


def make_spool(z_lid):
    """Ovaler Wickelkern mit Haltelaschen (für das IPEX-SMA-Kabel)."""
    zb = z_lid - SPOOL_H
    outer = rrect(SPOOL_X0, SPOOL_Y0, SPOOL_X1, SPOOL_Y1, SPOOL_R - 0.01)
    inner = rrect(SPOOL_X0 + SPOOL_WALL, SPOOL_Y0 + SPOOL_WALL, SPOOL_X1 - SPOOL_WALL,
                  SPOOL_Y1 - SPOOL_WALL, SPOOL_R - SPOOL_WALL - 0.01)
    core = (outer - inner).extrude(SPOOL_H + 0.01).translate([0, 0, zb])

    # Haltelaschen: unten am Kern nach außen, Oberseite 45° (druckt ohne Support)
    def tab(px, py, ang):
        w, t = SPOOL_TAB_W, 0.5
        a = box(-0.6, -w / 2, zb, SPOOL_TAB, w / 2, zb + t)
        b = box(-0.6, -w / 2, zb, -0.05, w / 2, zb + t + SPOOL_TAB)
        return Manifold.batch_hull([a, b]).rotate([0, 0, ang]).translate([px, py, 0])

    xa, xb = SPOOL_X0 + SPOOL_R, SPOOL_X1 - SPOOL_R     # Mittelpunkte der Rundungen
    yc = (SPOOL_Y0 + SPOOL_Y1) / 2
    tabs = []
    for x in (xa + (xb - xa) * 0.3, xa + (xb - xa) * 0.7):   # gerade Seiten
        tabs += [tab(x, SPOOL_Y0, -90), tab(x, SPOOL_Y1, 90)]
    for ang in (139, 182, 221):                         # Rundung USB-Ende
        r = np.radians(ang)
        tabs.append(tab(xa + SPOOL_R * np.cos(r), yc + SPOOL_R * np.sin(r), ang))
    for ang in (-41, 2, 41):                            # Rundung Antennen-Ende
        r = np.radians(ang)
        tabs.append(tab(xb + SPOOL_R * np.cos(r), yc + SPOOL_R * np.sin(r), ang))
    return core + Manifold.batch_boolean(tabs, OpType.Add)


# ---------------------------------------------------------------------------
# Dummy-Bauteile für die Kollisionsprüfung / Vorschau
# ---------------------------------------------------------------------------
def make_dummies():
    d = {}
    d["lora_pcb"] = box(LORA_X0, BOARD_Y0, LORA_Z, LORA_X1, BOARD_Y0 + BOARD_W, LORA_Z + LORA_PCB)
    pins = Manifold()
    for yy in (BOARD_Y0 + 1.61, BOARD_Y0 + BOARD_W - 1.61):
        for i in range(20):
            x = LORA_X0 + 1.37 + 0.5 + i * 2.54
            pins += box(x - 0.32, yy - 0.32, LORA_Z - PIN_BELOW, x + 0.32, yy + 0.32, LORA_Z)
    d["pins"] = pins
    hdr = Manifold()
    for yy in (BOARD_Y0 + 1.61, BOARD_Y0 + BOARD_W - 1.61):
        hdr += box(LORA_X0 + 0.5, yy - 1.27, LORA_Z + LORA_PCB, LORA_X0 + 51.3, yy + 1.27,
                   LORA_Z + LORA_PCB + FEMALE_HDR + MALE_PLASTIC)
    d["headers"] = hdr
    d["pico_pcb"] = box(PICO_X0, BOARD_Y0, PICO_Z, PICO_X0 + PICO_L, BOARD_Y0 + BOARD_W, PICO_TOP)
    d["usb"] = box(USB_FACE_X, BOARD_YC - 3.75, PICO_TOP, USB_FACE_X + 5.5, BOARD_YC + 3.75, PICO_TOP + 2.8)
    d["debug"] = box(PICO_X0 + 29.8, BOARD_Y0 + 10.5, PICO_TOP, PICO_X0 + 33.3, BOARD_Y0 + 15.5,
                     PICO_TOP + PICO_COMP_H)
    d["bootsel"] = box(PICO_X0 + 10, BOARD_Y0 + 4, PICO_TOP, PICO_X0 + 14, BOARD_Y0 + 8, PICO_TOP + 2.5)
    d["battery"] = box(BAY_X0 + 0.3, BAY_Y0 + BAT_CLR / 2, 0, BAY_X0 + 0.3 + BAT_L,
                       BAY_Y0 + BAT_CLR / 2 + BAT_FOOT, BAT_H)
    # Schiebeschalter unter dem LoRa-Board, Schieber ragt Richtung USB-Ende (hier: OFF)
    zs = LORA_Z - SW_BODY_H
    d["switch"] = (box(LORA_X0 + SW_BODY_X0, BOARD_Y0 + SW_BODY_Y0, zs, LORA_X0 + SW_BODY_X1,
                       BOARD_Y0 + SW_BODY_Y1, LORA_Z)
                   + box(LORA_X0 - 0.5, BOARD_Y0 + SW_SLIDER_OFF - SW_SLIDER_W / 2, zs + 0.4,
                         LORA_X0 + SW_BODY_X0, BOARD_Y0 + SW_SLIDER_OFF + SW_SLIDER_W / 2, LORA_Z - 0.4))
    # Akkubuchse + Stecker + Kabel Richtung Akkufach
    d["bat_conn"] = (box(LORA_X0 + CONN_X0, BOARD_Y0 + CONN_Y0, LORA_Z - 3.4, LORA_X0 + CONN_X1,
                         BOARD_Y0 + CONN_Y1, LORA_Z)
                     + box(LORA_X0 + CONN_X0 + 1.5, BOARD_Y0 + CONN_Y1, LORA_Z - 3.0, LORA_X0 + CONN_X1 - 1.5,
                           BOARD_Y0 + PLUG_Y1, LORA_Z - 0.2))
    # Akkukabel: Stecker -> zwischen den Pins durch -> am Platinenrand entlang nach vorn
    # -> quer hinüber zum Akku-Kopf
    edge_y0, edge_y1 = BOARD_Y0 + BOARD_W - 1.29 + 0.15, DIV_Y0 - 0.1
    d["bat_wires"] = (box(LORA_X0 + CABLE_X0, BOARD_Y0 + PLUG_Y1, LORA_Z - 2.6, LORA_X0 + CABLE_X1,
                          edge_y1, LORA_Z - 0.6)
                      + box(1.0, edge_y0, 1.5, LORA_X0 + CABLE_X1, edge_y1, LORA_Z - 0.6)
                      + box(1.0, edge_y0, 3.2, BAY_X0 - 0.4, BAY_Y0 + BAY_W / 2, 6.0))   # über den vorderen Anschlag
    # SMA-Buchse innen: Mutter + Körper
    d["sma"] = (cyl(IN_L - SMA_NUT_L, SMA_Y, SMA_Z, SMA_NUT_L, 9.2, axis="x")            # Mutter + Körper
                + cyl(IN_L - SMA_RIGID, SMA_Y, SMA_Z, SMA_RIGID - SMA_NUT_L, 3.6, axis="x"))  # Crimphülse
    # IPEX-Buchse auf dem LoRa-Board (Antennen-Ende, Pin-1-Seite, unter dem Pico)
    d["ipex"] = box(LORA_X0 + 46.2, BOARD_Y0 + 3.6, LORA_Z + LORA_PCB, LORA_X0 + 49.2,
                    BOARD_Y0 + 6.6, LORA_Z + LORA_PCB + 1.5)
    # typischer Micro-USB-Stecker (Umspritzung 12 x 9 mm)
    d["usb_plug"] = (box(-6.6 + USB_FACE_X, BOARD_YC - 3.5, USB_CZ - 0.9, USB_FACE_X, BOARD_YC + 3.5, USB_CZ + 0.9)
                     + box(-25, BOARD_YC - 6, USB_CZ - 4.5, -6.6 + USB_FACE_X, BOARD_YC + 6, USB_CZ + 4.5))
    return d


def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=False)


def main(out_dir=None):
    import os
    if out_dir is None:   # Standard: Ordner "stl" neben diesem Skript
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
    os.makedirs(out_dir, exist_ok=True)
    box_m = make_box()
    lid_m = make_lid()

    # Export: Unterteil auf z=0 stellen; Deckel umdrehen (Platte aufs Bett)
    box_exp = box_m.translate([WALL, WALL, FLOOR])
    lid_exp = lid_m.rotate([180, 0, 0]).translate([WALL, IN_W + WALL, IN_H + LID_T])
    to_trimesh(box_exp).export(os.path.join(out_dir, "pico_lora_box.stl"))
    to_trimesh(lid_exp).export(os.path.join(out_dir, "pico_lora_lid.stl"))

    print(f"Außenmaße: {OUT_L:.1f} x {OUT_W:.1f} x {BOX_H + LID_T:.1f} mm (L x B x H)")
    print(f"Innenraum: {IN_L:.1f} x {IN_W:.1f} x {IN_H:.1f} mm")
    print(f"LoRa-Unterkante z={LORA_Z:.1f}, Pico-Oberkante z={PICO_TOP:.1f}, USB-Mitte z={USB_CZ:.1f}")
    return box_m, lid_m


if __name__ == "__main__":
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else None)
