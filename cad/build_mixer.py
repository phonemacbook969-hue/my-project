"""
Ikki valli (twin-shaft) beton qorishtirgich qozoni, 3000/2000 L (2 m3 zichlangan beton).
Parametrik B-rep CAD modeli: CadQuery / OpenCascade.

Chiqish fayllari:
  out/beton_qozon_2m3.step   - SolidWorks / Inventor / FreeCAD da ochiladigan yig'ma (assembly)
  viewer/mixer.json         - 3D ko'ruvchi uchun detallar ro'yxati va uchburchak to'rlari (base64)

O'lchamlar mm da. X - ko'ndalang, Y - vallar bo'ylab, Z - yuqoriga. Vallar o'qi z = 0.
"""
import json, math, os, struct, sys
import cadquery as cq
from cadquery import Workplane as W, Vector, Location

# ----------------------------------------------------------------- parametrlar
C = 430.0        # val o'qidan markazgacha (vallar orasi 860)
R = 600.0        # astar ichki radiusi
TL = 20.0        # Ni-Hard astar qalinligi
TS = 10.0        # korpus po'lati
RS = R + TL      # korpus ichki radiusi 620
RO = RS + TS     # korpus tashqi radiusi 630
LH = 900.0       # ichki uzunlikning yarmi (1800)
TE = 15.0        # yon devor astari
YW = LH + TE     # torets devor ichki yuzi 915
YO = YW + 20.0   # torets devor tashqi yuzi 935
HT = 700.0       # yuqori quti balandligi
RT = 594.0       # kurakcha uchi radiusi (astargacha 6 mm)
GZ = -990.0      # bo'shatish qopqog'i o'qi
GR = 400.0       # qopqoq radiusi
MW = 196.0       # bo'shatish og'zi yarim eni
ML = 600.0       # bo'shatish og'zi yarim uzunligi
MZ = 950.0       # motor o'qi balandligi
PITCH = 225.0    # qo'llar qadami
NARM = 8

parts = []
groups = {
    "static": {"pivot": [0, 0, 0], "axis": [0, 1, 0]},
    "shaft1": {"pivot": [-C, 0, 0], "axis": [0, 1, 0]},
    "shaft2": {"pivot": [C, 0, 0], "axis": [0, 1, 0]},
    "gpul1": {"pivot": [-C, 0, 0], "axis": [0, 1, 0]},
    "gpul2": {"pivot": [C, 0, 0], "axis": [0, 1, 0]},
    "mpul1": {"pivot": [-C, 0, MZ], "axis": [0, 1, 0]},
    "mpul2": {"pivot": [C, 0, MZ], "axis": [0, 1, 0]},
    "sync": {"pivot": [0, 1340, 0], "axis": [1, 0, 0]},
    "gate": {"pivot": [0, 0, GZ], "axis": [0, 1, 0]},
    "cylbody": {"pivot": [820, -765, -1100], "axis": [0, 1, 0]},
    "cylrod": {"pivot": [0, -765, GZ], "axis": [0, 1, 0]},
    "hatch1": {"pivot": [955, 0, 716], "axis": [0, 1, 0]},
    "hatch2": {"pivot": [-955, 0, 716], "axis": [0, 1, 0]},
}


# ----------------------------------------------------------------- yordamchilar
def cylY(r, y0, y1, x=0, z=0):
    return W("XY").circle(r).extrude(y1 - y0).rotate((0, 0, 0), (1, 0, 0), 90).translate((x, y1, z))


def cylX(r, x0, x1, y=0, z=0):
    return W("YZ").circle(r).extrude(x1 - x0).translate((x0, y, z))


def cylZ(r, z0, z1, x=0, y=0):
    return W("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


def box(x0, x1, y0, y1, z0, z1):
    return W("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def profY(pts, y0, y1):
    """(x, z) ko'pburchakni Y bo'ylab cho'zish."""
    return W("XY").polyline(pts).close().extrude(y1 - y0).rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y1, 0))


def profX(pts, x0, x1):
    """(y, z) ko'pburchakni X bo'ylab cho'zish."""
    return W("YZ").polyline(pts).close().extrude(x1 - x0).translate((x0, 0, 0))


def sector(r0, r1, a0, a1, y0, y1):
    """Halqa sektori; burchaklar X->Z yo'nalishida, gradus."""
    s = W("XY").circle(r1).extrude(y1 - y0)
    if r0 > 0:
        s = s.cut(W("XY").circle(r0).extrude(y1 - y0))
    rb = 3 * r1
    am = (a0 + a1) / 2
    pts = [(0, 0)] + [(rb * math.cos(math.radians(a)), rb * math.sin(math.radians(a))) for a in (a0, am, a1)]
    s = s.intersect(W("XY").polyline(pts).close().extrude(y1 - y0))
    return s.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y1, 0))


def omega(r, halfw, ztop, y0, y1):
    """Ikki silindr + yuqori quti: qozonning 'omega' kesimi."""
    return cylY(r, y0, y1, x=-C).union(cylY(r, y0, y1, x=C)).union(box(-halfw, halfw, y0, y1, 0, ztop))


def hull2(r1, r2, d, y0, y1):
    """Ikki aylana qobig'i (tasma konturi): (0,0) da r1, (0,d) da r2."""
    sk = cq.Sketch().arc((0, 0), r1, 0.0, 360.0).arc((0, d), r2, 0.0, 360.0).hull()
    return W("XY").placeSketch(sk).extrude(y1 - y0).rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y1, 0))


def hexX(af, x0, x1, y=0, z=0):
    return W("YZ").polygon(6, af / math.cos(math.radians(30))).extrude(x1 - x0).translate((x0, y, z))


def hexY(af, y0, y1, x=0, z=0):
    return (W("XY").polygon(6, af / math.cos(math.radians(30))).extrude(y1 - y0)
            .rotate((0, 0, 0), (1, 0, 0), 90).translate((x, y1, z)))


def hexZ(af, z0, z1, x=0, y=0):
    return W("XY").polygon(6, af / math.cos(math.radians(30))).extrude(z1 - z0).translate((x, y, z0))


def comp(*wps):
    """Birlashtirmasdan kompaund (mayda detallar uchun tez)."""
    sh = []
    for w in wps:
        sh.extend(w.vals() if isinstance(w, W) else [w])
    return cq.Compound.makeCompound(sh)


def pipe(pts, r):
    """Nuqtalar bo'yicha to'g'ri bo'lakli quvur."""
    sh = []
    for a, b in zip(pts[:-1], pts[1:]):
        va, vb = Vector(*a), Vector(*b)
        d = vb - va
        sh.append(cq.Solid.makeCylinder(r, d.Length, va, d))
    for p in pts[1:-1]:
        sh.append(cq.Solid.makeSphere(r, Vector(*p)))
    return cq.Compound.makeCompound(sh)


def T(x=0, y=0, z=0):
    return Location(Vector(x, y, z))


def RY(theta):
    """Y o'qi atrofida X->Z yo'nalishida theta gradus."""
    return Location(Vector(0, 0, 0), Vector(0, 1, 0), -theta)


def RX(a):
    return Location(Vector(0, 0, 0), Vector(1, 0, 0), a)


def RZ(a):
    return Location(Vector(0, 0, 0), Vector(0, 0, 1), a)


def add(pid, name, desc, mat, shape, group="static", inst=None, ang=0.25, tol=0.8, tag=""):
    if isinstance(shape, W):
        vals = shape.vals()
        shape = vals[0] if len(vals) == 1 else cq.Compound.makeCompound(vals)
    parts.append(dict(id=pid, name=name, desc=desc, mat=mat, shape=shape, group=group,
                      inst=inst or [Location()], ang=ang, tol=tol, tag=tag))
    print("  +", pid, flush=True)


# ================================================================= KORPUS
print("korpus")
shell = omega(RO, C + RO, HT, -YO, YO).cut(omega(RS, C + RS, HT + 10, -YW, YW))
shell = shell.cut(box(-MW, MW, -ML, ML, -720, -300))
for sx in (-C, C):
    shell = shell.cut(cylY(105, -YO - 5, YO + 5, x=sx))
# yuqori flanes
shell = shell.union(box(-1130, 1130, -1005, 1005, 688, 700).cut(box(-C - RS, C + RS, -YW, YW, 680, 705)))
# halqa qovurg'alar
for y in (-850, -300, 300, 850):
    rib = omega(690, 1120, 690, y - 6, y + 6).cut(omega(RO - 1, C + RO - 1, 720, y - 10, y + 10))
    if abs(y) < 600:
        rib = rib.cut(box(-260, 260, y - 10, y + 10, -800, -350))
    shell = shell.union(rib)
for sx in (-1, 1):
    shell = shell.union(box(sx * 1060 if sx > 0 else -1120, 1120 if sx > 0 else -1060, -YO, YO, 344, 356))
add("korpus", "Qozon korpusi",
    "Ikki silindrli (omega kesimli) payvand korpus. Po'lat S355, devor 10 mm, torets devorlari 20 mm. "
    "Ichki o'lchami 2060 x 1800 mm, geometrik hajmi ~4,5 m3 (3000 L quruq yuklash / 2000 L zichlangan beton).",
    "body", shell, ang=0.12)

# tayanch oyoqlari
foot = box(1056, 1340, -150, 150, -300, -275)
gus = [(1058, 350), (1320, -275), (1058, -275)]
foot = foot.union(profY(gus, -132, -120)).union(profY(gus, 120, 132))
for hx in (1130, 1290):
    for hy in (-100, 100):
        foot = foot.cut(cylZ(13, -310, -270, x=hx, y=hy))
footL = foot.mirror("YZ")
add("oyoq_r", "Tayanch oyog'i", "Qozonni platforma to'sinlariga 4 ta nuqtada mahkamlaydi. Asos plitasi 25 mm, "
    "M24 ankrovka boltlari, tebranish yostiqchasi ustiga o'rnatiladi.", "dark", foot,
    inst=[T(0, -600, 0), T(0, 600, 0)])
add("oyoq_l", "Tayanch oyog'i", "Qozonni platforma to'sinlariga 4 ta nuqtada mahkamlaydi. Asos plitasi 25 mm, "
    "M24 ankrovka boltlari, tebranish yostiqchasi ustiga o'rnatiladi.", "dark", footL,
    inst=[T(0, -600, 0), T(0, 600, 0)])

# bo'shatish og'zi ramkasi
mouth = (box(-MW - 12, MW + 12, -ML - 12, ML + 12, -720, -380)
         .cut(box(-MW, MW, -ML, ML, -730, -370))
         .cut(cylY(GR + 2, -700, 700, z=GZ))
         .cut(cylY(RS, -700, 700, x=-C)).cut(cylY(RS, -700, 700, x=C)))
add("ogiz", "Bo'shatish og'zi ramkasi",
    "Ikki silindr tutashgan joydagi 1200 x 392 mm bo'shatish teshigi. Pastki qirrasi sektor qopqoq radiusiga "
    "(R400) mos kesilgan, almashtiriladigan zichlagich planka bilan.", "body", mouth, ang=0.15)

# ================================================================= ASTARLAR
print("astarlar")
SPAN = 180.0 - math.degrees(math.acos(C / R)) + 0.0   # 180 -> 315.8 = 135.8 gradus
SPAN = 360.0 - math.degrees(math.acos(C / R)) - 180.0
NT = 6
DA = SPAN / NT
tile = sector(R, RS, -DA / 2 + 0.15, DA / 2 - 0.15, -148, 148)
for hy in (-75, 75):
    tile = tile.cut(cylX(15, R - 1, R + 7, y=hy))
tinst, ninst = [], []
for side in (-1, 1):
    for k in range(NT):
        th = 180 + (k + 0.5) * DA
        if side > 0:
            th = (180 - th) % 360
        for row in range(6):
            yc = -750 + 300 * row
            if k == NT - 1 and abs(yc) < ML:
                continue
            tinst.append(T(side * C, yc, 0) * RY(th))
            for hy in (-75, 75):
                ninst.append(T(side * C, yc + hy, 0) * RY(th) * T(RO, 0, 0))
add("astar_tag", "Tag astari (Ni-Hard 4)",
    "Silindr tubining almashtiriladigan quyma plitkalari, 20 mm, qattiqligi 550-600 HB. "
    "Har bir plitka 2 ta yashirin kallakli bolt bilan tashqaridan tortiladi. Jami 64 dona.",
    "liner", tile, inst=tinst, ang=0.12)
nut = comp(hexX(30, 0, 16), cylX(10, 0, 26))
add("astar_gayka", "Astar gaykasi M20",
    "Tag plitkalarini korpus tashqarisidan tortadigan o'z-o'zidan qotuvchi gaykalar. "
    "Plitka yeyilganda ichkaridan emas, tashqaridan yechiladi.", "steel", nut, inst=ninst, ang=0.6)

el = omega(R, C + R + 5, 500, LH, YW)
for sx in (-C, C):
    el = el.cut(cylY(122, LH - 5, YW + 5, x=sx))
el = el.cut(box(-2, 2, LH - 5, YW + 5, -460, 520)).cut(box(-1100, 1100, LH - 5, YW + 5, -2, 2))
for sx in (-C, C):
    el = el.cut(box(sx - 2, sx + 2, LH - 5, YW + 5, -620, -122))
add("astar_torets", "Torets devor astari",
    "Torets devorlarining segmentli yeyilishga chidamli plitalari (Hardox 500, 15 mm). "
    "Val o'tgan joyda labirint halqa uchun teshik.", "liner", el,
    inst=[Location(), RZ(180)], ang=0.12)
sl = comp(*[box(C + R + 5, C + RS, y0, y1, 4, 500) for (y0, y1) in ((-898, -302), (-298, 298), (302, 898))])
add("astar_yon", "Yon devor astari", "Yuqori qutining yon devorlarini qoplaydigan plitalar (Hardox 500, 15 mm), "
    "har tomonda 3 tadan.", "liner", sl, inst=[Location(), RZ(180)])

# ================================================================= VALLAR
print("vallar")
shaft = (W("XY").polygon(6, 173).extrude(1760).rotate((0, 0, 0), (1, 0, 0), 90).translate((0, 880, 0))
         .union(cylY(70, -1190, -878)).union(cylY(70, 878, 1215))
         .union(cylY(118, 880, 899)).union(cylY(118, -899, -880)))
add("val", "Qorishtirish vali (olti qirrali)",
    "Olti qirrali val, qirralar orasi 150 mm, 42CrMo4 po'latdan. Olti qirra qo'llarning valda sirpanmasligini "
    "ta'minlaydi. Aylanish tezligi ~25,7 ayl/min, ikki val bir-biriga qarama-qarshi aylanadi.",
    "steel", shaft, group="shaft1", ang=0.2)
parts.append(dict(parts[-1], id="val2", group="shaft2"))


def make_hub():
    h = cylY(140, -70, 70).union(box(-62, 62, -70, 70, -168, -95)).union(box(-62, 62, -70, 70, 95, 168))
    h = h.cut(W("XY").polygon(6, 175).extrude(200).rotate((0, 0, 0), (1, 0, 0), 90).translate((0, 100, 0)))
    h = h.cut(box(-2, 2, -80, 80, -200, 200))
    b = []
    for by in (-35, 35):
        for bz in (-132, 132):
            b.append(hexX(30, 62, 76, y=by, z=bz))
            b.append(hexX(30, -76, -62, y=by, z=bz))
    return h, b


def make_arm(tau, r_in, width, yoff):
    h, b = make_hub()
    bar = box(120, 450, -30, 30, -40, 40)
    if yoff:
        bar = profY([(120, -40), (450, -40), (450, 40), (120, 40)], -30, 30)
        bar = box(120, 300, -30, 30, -40, 40).union(box(260, 450, min(0, yoff) - 30, max(0, yoff) + 30, -40, 40))
    pad = box(r_in, R + 20, -width / 2, width / 2, -15, 15).union(box(r_in + 20, 470, -70, 70, -48, -15))
    bolts = [W("XY").polygon(6, 36).extrude(12).translate((r_in + 60, sy * 60, 15)) for sy in (-1, 1)]
    bolts += [W("XY").polygon(6, 36).extrude(12).translate((r_in + 60, sy * 60, -60)) for sy in (-1, 1)]
    pad = pad.rotate((0, 0, 0), (1, 0, 0), tau).translate((0, yoff, 0)).intersect(cylY(RT, -400, 400))
    bolts = [x.rotate((0, 0, 0), (1, 0, 0), tau).translate((0, yoff, 0)) for x in bolts]
    body = h.union(bar)
    return body, pad, b + bolts


arm_body, arm_pad, arm_bolts = make_arm(45, 380, 260, 0)
scr_body, scr_pad, scr_bolts = make_arm(70, 290, 200, 58)

ys = [-(NARM - 1) / 2 * PITCH + PITCH * i for i in range(NARM)]
a1 = [90.0 * i for i in range(NARM)]
a2 = [270.0 - 90.0 * i for i in range(NARM)]
main1 = [T(0, ys[i], 0) * RY(a1[i]) for i in range(1, NARM - 1)]
main2 = [T(0, ys[i], 0) * RY(a2[i]) for i in range(1, NARM - 1)]
MIRY = None  # XZ tekisligida ko'zgu alohida to'r sifatida
scrN_body, scrN_pad = scr_body.mirror("XZ"), scr_pad.mirror("XZ")
scrN_bolts = [x.mirror("XZ") for x in scr_bolts]
# 1-val: +y uchida scr (P), -y uchida scrN.  2-val: +y uchida RX(180)*scrN, -y uchida RX(180)*scr
sP1 = [T(0, ys[-1], 0) * RY(a1[-1])]
sN1 = [T(0, ys[0], 0) * RY(a1[0])]
sP2 = [T(0, ys[0], 0) * RY(a2[0]) * RX(180)]     # scr to'ri, 180 burilgan -> -y uchi
sN2 = [T(0, ys[-1], 0) * RY(a2[-1]) * RX(180)]   # scrN to'ri, 180 burilgan -> +y uchi

D_ARM = ("Quyma po'lat qo'l (GS-52), ikki pallali xomut bilan olti qirrali valga 4 ta M20 bolt orqali siqiladi. "
         "Har bir valda 6 ta qorishtirish va 2 ta torets qo'li, qo'shni qo'llar 90 gradusga burilgan.")
D_PAD = ("Ni-Hard 4 quymasidan 30 mm kurakcha, valga nisbatan 45 gradus burilgan: beton bir valda oldinga, "
         "ikkinchisida orqaga suriladi va qozon ichida aylanma oqim hosil bo'ladi. Astargacha tirqish 3-6 mm, "
         "yeyilganda bolt teshiklari bo'yicha suriladi.")
D_SCR = ("Torets devoriga deyarli parallel qo'yilgan qirg'ich kurakcha. Devorga yopishgan betonni qirib, "
         "oqimni qozon ichkarisiga qaytaradi. Ni-Hard 4, 30 mm.")
D_BLT = "Qo'l xomuti va kurakchani mahkamlovchi yuqori mustahkamlikdagi boltlar (10.9 sinf)."
for g, mi, sp, sn in (("shaft1", main1, sP1, sN1), ("shaft2", main2, sN2, sP2)):
    # sp: scr to'ri uchun, sn: scrN to'ri uchun joylashuvlar
    n = g[-1]
    add("qol" + n, "Qorishtirish qo'li", D_ARM, "castarm", arm_body, group=g, inst=mi, ang=0.3)
    add("kurak" + n, "Qorishtirish kurakchasi", D_PAD, "nihard", arm_pad, group=g, inst=mi, ang=0.15)
    add("qolbolt" + n, "Qo'l va kurakcha boltlari", D_BLT, "steel", comp(*arm_bolts), group=g, inst=mi, ang=0.6)
    add("tqol" + n + "a", "Torets qo'li", D_ARM, "castarm", scr_body, group=g, inst=sp, ang=0.3)
    add("tkurak" + n + "a", "Torets qirg'ich kurakchasi", D_SCR, "nihard", scr_pad, group=g, inst=sp, ang=0.15)
    add("tbolt" + n + "a", "Qo'l va kurakcha boltlari", D_BLT, "steel", comp(*scr_bolts), group=g, inst=sp, ang=0.6)
    add("tqol" + n + "b", "Torets qo'li", D_ARM, "castarm", scrN_body, group=g, inst=sn, ang=0.3)
    add("tkurak" + n + "b", "Torets qirg'ich kurakchasi", D_SCR, "nihard", scrN_pad, group=g, inst=sn, ang=0.15)
    add("tbolt" + n + "b", "Qo'l va kurakcha boltlari", D_BLT, "steel", comp(*scrN_bolts), group=g, inst=sn, ang=0.6)

# ================================================================= ZICHLAGICH, PODSHIPNIK
print("podshipnik uzellari")
ends = [T(sx, 0, 0) * (Location() if sy > 0 else RZ(180)) for sx in (-C, C) for sy in (1, -1)]
seal = cylY(150, YO, 990).union(cylY(178, YO, YO + 16)).cut(cylY(72, YO - 5, 995))
sb = [hexY(24, YO + 16, YO + 27, x=165 * math.cos(math.radians(a)), z=165 * math.sin(math.radians(a)))
      for a in range(0, 360, 45)]
seal = comp(seal, *sb, cylZ(6, 150, 172, y=965), hexZ(14, 172, 182, y=965))
add("salnik", "Val zichlagichi (ko'p pog'onali)",
    "Val korpusdan chiqadigan joydagi labirint + suzuvchi halqali zichlagich. Markaziy moylash tizimi "
    "uzluksiz konsistent moy haydab, sement suti podshipnikka o'tishining oldini oladi. "
    "Podshipnikdan havo oralig'i (fonar) bilan ajratilgan.", "gear", seal, inst=ends, ang=0.3)
lant = (cylY(200, YO, 1070).cut(cylY(186, YO - 5, 1075))
        .cut(box(-300, 300, 965, 1045, -62, 62)).cut(box(-62, 62, 965, 1045, -300, 300))
        .union(cylY(232, 1060, 1080).cut(cylY(100, 1050, 1090))))
add("fonar", "Podshipnik fonari",
    "Zichlagich bilan podshipnik orasidagi ochiq derazali tayanch. Zichlagichdan sizgan suyuqlik derazadan "
    "tashqariga oqib ketadi va podshipnikka yetib bormaydi.", "body", lant, inst=ends, ang=0.25)
brg = cylY(190, 1080, 1170).union(cylY(120, 1170, 1186)).cut(cylY(72, 1075, 1190))
bb = [hexY(30, 1170, 1184, x=160 * math.cos(math.radians(a)), z=160 * math.sin(math.radians(a)))
      for a in range(30, 360, 60)]
add("podshipnik", "Val podshipnik uzeli",
    "Ikki qatorli sferik rolikli podshipnik (22228 turi) flanesli korpusda. Val egilishini o'zi "
    "to'g'rilaydi. Har bir valda 2 tadan, jami 4 ta.", "gear", comp(brg, *bb), inst=ends, ang=0.3)

# ================================================================= REDUKTOR, YURITMA
print("yuritma")
gb = cylY(215, 1185, 1200).union(cylY(262, 1200, 1226)).union(cylY(240, 1226, 1500)) \
    .union(cylY(256, 1290, 1410)).union(cylY(150, 1500, 1558)).union(cylY(262, 1474, 1500))
gb = gb.union(cylX(78, 150, 305, y=1340)).union(cylX(95, 290, 305, y=1340))
fins = [box(250, 268, 1296, 1404, -5, 5).rotate((0, 0, 0), (0, 1, 0), a) for a in range(20, 360, 20)
        if not (a < 30 or a > 330)]
gbb = [hexY(30, 1186, 1200, x=242 * math.cos(math.radians(a)), z=242 * math.sin(math.radians(a)))
       for a in range(15, 360, 30)]
gbb += [hexY(30, 1500, 1513, x=242 * math.cos(math.radians(a)), z=242 * math.sin(math.radians(a)))
        for a in range(15, 360, 30)]
add("reduktor", "Planetar reduktor",
    "Valga bevosita o'rnatiladigan ikki pog'onali planetar reduktor, uzatish soni ~1:28,8, chiqish momenti "
    "~14 kNm. Yon tomondagi konussimon uzatma sinxronlash valiga ulanadi. Moy vannasida ishlaydi.",
    "gearblue", comp(gb, *fins, *gbb), inst=[T(-C, 0, 0), T(C, 0, 0) * RY(180)], ang=0.25)

sync = comp(cylX(25, -125, 125), cylX(62, -125, -70), cylX(62, 70, 125), cylX(40, -70, -50), cylX(40, 50, 70),
            box(-110, -85, -8, 8, 55, 68), box(85, 110, -8, 8, -68, -55))
add("sinxron", "Sinxronlash vali",
    "Ikki reduktorning tez yuradigan pog'onalarini elastik muftalar orqali bog'laydi. Vallar fazasini "
    "qat'iy ushlab turadi: kurakchalar kesishish zonasida hech qachon to'qnashmaydi; bitta motor "
    "to'xtasa ham ikkala val birga aylanadi.", "steel", sync, group="sync", ang=0.3)


def pulley(r, bore, y0, holes):
    pts = [(bore, 0), (r, 0)]
    for k in range(5):
        g0 = 4 + k * 15
        pts += [(r, g0), (r - 12, g0 + 4.5), (r - 12, g0 + 8.5), (r, g0 + 13)]
    pts += [(r, 80), (bore, 80)]
    p = W("XY").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0)).translate((0, y0, 0))
    if holes:
        for a in range(0, 360, 60):
            p = p.cut(cylY(38, y0 - 5, y0 + 85, x=150 * math.cos(math.radians(a)), z=150 * math.sin(math.radians(a))))
        p = p.cut(cylY(215, y0 + 55, y0 + 85).cut(cylY(75, y0 + 50, y0 + 90)))
    return p


gp = comp(pulley(250, 30, 1565, True), cylY(30, 1555, 1652), cylY(55, 1645, 1660), box(-8, 8, 1650, 1662, 20, 40))
add("shkiv_r", "Reduktor shkivi (D500, 5xSPB)",
    "Yetaklanuvchi ponasimon tasmali shkiv. Motor shkivi bilan nisbati 2:1 — reduktor kirishida 740 ayl/min.",
    "castarm", gp, group="gpul1", ang=0.2)
parts.append(dict(parts[-1], id="shkiv_r2", group="gpul2"))
mp = comp(pulley(125, 30, 1565, False), cylY(30, 1530, 1652), cylY(48, 1645, 1658), box(-8, 8, 1650, 1660, 15, 34))
add("shkiv_m", "Motor shkivi (D250, 5xSPB)",
    "Yetakchi shkiv, konussimon vtulka bilan motor valiga o'rnatilgan. 1480 ayl/min.",
    "castarm", mp, group="mpul1", ang=0.25)
parts.append(dict(parts[-1], id="shkiv_m2", group="mpul2"))

belt = hull2(254, 129, MZ, 0, 11).cut(hull2(243, 118, MZ, -1, 12))
add("tasma", "Ponasimon tasmalar (5 x SPB)",
    "Har bir motorda 5 ta tor profilli ponasimon tasma. Ishga tushishda zarbani yumshatadi va reduktorni "
    "tiqilib qolishdan himoya qiladi. Motor plitasidagi shpilkalar bilan taranglanadi.",
    "rubber", belt, inst=[T(sx, 1565 + 5 + 15 * k + 11, 0) for sx in (-C, C) for k in range(5)], ang=0.15)
guard = hull2(278, 153, MZ, 1548, 1668).cut(hull2(275, 150, MZ, 1540, 1665))
add("gilof", "Tasma g'ilofi",
    "Aylanuvchi shkiv va tasmalarni to'liq yopadigan himoya g'ilofi (xavfsizlik talabi). "
    "Ichini ko'rish uchun paneldan yashirish mumkin.", "yellow", guard,
    inst=[T(-C, 0, 0), T(C, 0, 0)], ang=0.15, tag="guard")

# motor
mb = cylY(215, 1040, 1500).union(cylY(224, 1500, 1528)).union(cylY(224, 1026, 1040)) \
    .union(cylY(62, 1528, 1544))
mfins = [box(210, 234, 1050, 1490, -3, 3).rotate((0, 0, 0), (0, 1, 0), a) for a in range(0, 360, 10)
         if not (60 < a < 120)]
mfeet = [box(-200, -110, 1090, 1420, -225, -205), box(110, 200, 1090, 1420, -225, -205),
         box(-170, -150, 1100, 1410, -210, -120), box(150, 170, 1100, 1410, -210, -120)]
mfb = [hexZ(30, -205, -192, x=sx * 155, y=yy) for sx in (-1, 1) for yy in (1120, 1390)]
add("motor", "Elektr motor 37 kVt",
    "Uch fazali asinxron motor, 37 kVt, 400 V, 1480 ayl/min, IE3, gabarit 225S, himoya IP55. "
    "Har bir valga bittadan — jami 2 x 37 = 74 kVt. Yumshoq ishga tushirish (yulduz-uchburchak) bilan.",
    "motor", comp(mb, *mfins, *mfeet, *mfb), inst=[T(-C, 0, MZ), T(C, 0, MZ)], ang=0.25)
cowl = cylY(228, 936, 1040).cut(cylY(222, 946, 1045))
for rr, n in ((60, 6), (120, 12), (180, 18)):
    for i in range(n):
        a = 2 * math.pi * i / n
        cowl = cowl.cut(cylY(16, 930, 950, x=rr * math.cos(a), z=rr * math.sin(a)))
add("motor_qopqoq", "Motor ventilyator qopqog'i",
    "Sovutish ventilyatorini yopadigan panjarali qopqoq. Ventilyator havoni korpus qovurg'alari bo'ylab haydaydi.",
    "dark", cowl, inst=[T(-C, 0, MZ), T(C, 0, MZ)], ang=0.25)
tbx = comp(box(-110, 110, 1180, 1380, 205, 320), box(-118, 118, 1172, 1388, 320, 332),
           cylX(22, 110, 150, y=1280, z=262), box(-216, -212, 1200, 1340, -60, 40).rotate((0, 0, 0), (0, 1, 0), 0))
add("motor_klemma", "Klemma qutisi", "Motorning ta'minot kabeli ulanadigan klemma qutisi, PG salnik bilan.",
    "dark", tbx, inst=[T(-C, 0, MZ), T(C, 0, MZ)])

cons = (box(-230, 230, 1060, 1440, 707, 725)
        .union(box(-220, -140, YO, 1440, 580, 686)).union(box(140, 220, YO, 1440, 580, 686))
        .union(box(-220, -140, 1060, 1440, 686, 707)).union(box(140, 220, 1060, 1440, 686, 707))
        .union(profX([(YO, 580), (1310, 580), (YO, 300)], -186, -174))
        .union(profX([(YO, 580), (1310, 580), (YO, 300)], 174, 186)))
rods = [cylZ(10, 690, 775, x=sx * 205, y=yy) for sx in (-1, 1) for yy in (1085, 1415)]
rods += [hexZ(30, 745, 761, x=sx * 205, y=yy) for sx in (-1, 1) for yy in (1085, 1415)]
add("motor_konsol", "Motor konsoli va taranglash plitasi",
    "Torets devoriga payvandlangan konsol. Motor plitasi 4 ta shpilkada ko'tarilib-tushib, "
    "ponasimon tasmalar tarangligini rostlaydi.", "dark", comp(cons, *rods), inst=[T(-C, 0, 0), T(C, 0, 0)])

# ================================================================= USTKI QOPQOQ
print("ustki qism")
cover = box(-1130, 1130, -1005, 1005, 700, 708)
cover = cover.cut(box(-450, 450, -50, 650, 690, 720)).cut(cylZ(150, 690, 720, y=-520))
cover = cover.cut(cylZ(100, 690, 720, x=-760, y=600))
for sx in (-1, 1):
    cover = cover.cut(box(min(sx * 300, sx * 950), max(sx * 300, sx * 950), -850, -250, 690, 720))
    cover = cover.cut(cylZ(47, 690, 720, x=sx * 760, y=-150))
cbl = [hexZ(24, 708, 719, x=x, y=sy * 970) for x in range(-1050, 1051, 300) for sy in (-1, 1)]
cbl += [hexZ(24, 708, 719, x=sx * 1095, y=y) for y in range(-750, 751, 300) for sx in (-1, 1)]
stf = [box(-1050, 1050, -80, -68, 708, 748), box(-1050, -460, 668, 680, 708, 748), box(460, 1050, 668, 680, 708, 748)]
add("qopqoq", "Ustki qopqoq (chang o'tkazmaydigan)",
    "Qozonni yuqoridan yopadigan boltli qopqoq, 8 mm po'lat, rezina zichlagichda. Inert materiallar, sement, "
    "suv kirishlari, chang chiqarish patrubkasi va 2 ta ko'rish lyuki shu qopqoqda.", "body",
    comp(cover, *cbl, *stf), tag="cover")

chute = (box(-462, 462, -62, 662, 708, 1000).cut(box(-450, 450, -50, 650, 700, 1010))
         .union(box(-530, 530, -130, 730, 1000, 1012).cut(box(-450, 450, -50, 650, 990, 1020))))
add("inert_kirish", "Inert materiallar kirish bo'g'zi",
    "Skip ko'targich yoki lentali konveyerdan keladigan qum va shag'al shu 900 x 700 mm bo'g'iz orqali "
    "to'g'ridan-to'g'ri vallar ustiga tushadi. Ichi rezina bilan qoplangan.", "body", chute, tag="cover")
cem = cylZ(160, 708, 900).cut(cylZ(150, 700, 910)).union(cylZ(212, 900, 914).cut(cylZ(150, 890, 920)))
add("sement_kirish", "Sement kirish patrubkasi DN300",
    "Sement tarozi bunkeridan keladigan patrubka.", "body", cem.translate((0, -520, 0)), tag="cover")
bfv = comp(cylZ(172, 914, 972).cut(cylZ(150, 910, 980)), cylZ(212, 972, 986).cut(cylZ(150, 970, 990)),
           box(172, 300, -40, 40, 918, 968), box(300, 470, -65, 65, 895, 990), cylZ(148, 938, 946))
add("sement_zatvor", "Kapalak zatvor DN300 (pnevmatik)",
    "Sement tarozisi ostidagi pnevmoyuritmali kapalak zatvor. Tarozi dozani o'lchab bo'lgach ochiladi, "
    "sement 8-10 soniyada qozonga tushadi.", "gearblue", bfv.translate((0, -520, 0)), tag="cover")
add("sement_yeng", "Egiluvchan yeng",
    "Tarozi bunkerini qozondan tebranish bo'yicha ajratadigan brezent-rezina yeng: qozon titrashi tarozi "
    "ko'rsatkichiga ta'sir qilmaydi.", "rubber",
    cylZ(158, 986, 1160).cut(cylZ(150, 980, 1170)).translate((0, -520, 0)), tag="cover")
vent = comp(cylZ(104, 708, 860).cut(cylZ(98, 700, 870)), cylZ(118, 850, 866).cut(cylZ(98, 840, 870)))
add("chang_patrubka", "Chang chiqarish patrubkasi",
    "Yuklash paytida siqib chiqariladigan changli havo shu yerdan filtrga ketadi.", "body",
    vent.translate((-760, 600, 0)), tag="cover")
add("chang_filtr", "Chang filtri yengi",
    "Matoli filtr yeng: havoni chiqaradi, sement changini ushlab qoladi.", "fabric",
    cylZ(112, 866, 1260).union(cylZ(118, 1260, 1272)).translate((-760, 600, 0)), tag="cover")

# suv tizimi
wz = 880
water = comp(pipe([(1250, -150, wz), (-760, -150, wz), (-760, -150, 600)], 45),
             pipe([(760, -150, wz), (760, -150, 600)], 45),
             cylX(100, 1250, 1266, y=-150, z=wz), cylX(100, 1090, 1106, y=-150, z=wz),
             cylX(100, 1166, 1182, y=-150, z=wz))
add("suv_quvur", "Suv ta'minot quvuri DN80",
    "Suv tarozi idishidan keladigan magistral. Qopqoq ustidan o'tib, ikki tomondagi purkash kollektorlariga "
    "tarmoqlanadi.", "water", water, tag="cover")
wv = comp(cylX(62, 1106, 1166, y=-150, z=wz), box(1116, 1156, -190, -110, wz + 50, wz + 130),
          cylY(48, -260, -100, x=1136, z=wz + 150))
add("suv_zatvor", "Suv zatvori (pnevmatik)",
    "Suv dozasini qozonga qo'yib yuboradigan pnevmoyuritmali zatvor.", "gearblue", wv, tag="cover")
spray = []
for sx in (-1, 1):
    spray.append(pipe([(sx * 760, -800, 600), (sx * 760, 800, 600)], 30))
    for yy in range(-700, 701, 200):
        spray.append(cq.Solid.makeCone(14, 8, 45, Vector(sx * 760, yy, 600) + Vector(-sx * 18, 0, -18),
                                       Vector(-sx, 0, -1)))
    spray.append(cylY(36, -812, -800, x=sx * 760, z=600))
    spray.append(cylY(36, 800, 812, x=sx * 760, z=600))
add("suv_purkagich", "Suv purkash kollektori",
    "Qozon ichida vallar bo'ylab o'tgan 2 ta quvur, har birida 8 tadan yelpig'ichsimon soplo. Suvni butun "
    "uzunlik bo'ylab bir tekis purkaydi; sikl oxirida qozonni yuvishda ham ishlatiladi.", "water", comp(*spray))
addp = comp(pipe([(1000, -700, wz + 80), (1000, -150, wz + 80), (1000, -150, wz + 30)], 15),
            cylY(32, -712, -700, x=1000, z=wz + 80))
add("qoshimcha", "Kimyoviy qo'shimcha quvuri DN25",
    "Plastifikator va boshqa suyuq qo'shimchalar suv oqimiga shu yerdan qo'shiladi.", "steel", addp, tag="cover")

# lyuklar
hatch = comp(box(-665, 5, -860, -240, -8, -2), box(-640, -620, -640, -460, -2, 50),
             box(-640, -620, -640, -620, -2, 50), box(-640, -620, -480, -460, -2, 50),
             cylY(13, -800, -700, x=0, z=0), cylY(13, -400, -300, x=0, z=0),
             box(-600, -60, -556, -544, -2, 22), box(-340, -328, -820, -280, -2, 22))
add("lyuk", "Ko'rish lyuki",
    "Astar va kurakchalarni ko'zdan kechirish, almashtirish uchun sharnirli lyuk, 650 x 600 mm. "
    "Xavfsizlik kaliti bilan bloklangan: lyuk ochilsa motorlar o'chadi.", "body", hatch, group="hatch1", tag="cover")
add("lyuk2", "Ko'rish lyuki",
    "Astar va kurakchalarni ko'zdan kechirish, almashtirish uchun sharnirli lyuk, 650 x 600 mm. "
    "Xavfsizlik kaliti bilan bloklangan: lyuk ochilsa motorlar o'chadi.", "body",
    cq.Compound.makeCompound(hatch.Solids()).mirror("YZ"), group="hatch2", tag="cover")
grill = [box(300, 950, -850 + 75 * i - 5, -850 + 75 * i + 5, 690, 700) for i in range(1, 8)]
grill += [box(300, 950, -850, -840, 684, 700), box(300, 950, -260, -250, 684, 700),
          box(300, 310, -850, -250, 684, 700), box(940, 950, -850, -250, 684, 700), box(620, 630, -850, -250, 688, 698)]
add("panjara", "Lyuk himoya panjarasi",
    "Lyuk ochiq bo'lganda ham odam qo'li yoki oyog'i qozon ichiga tushmasligi uchun payvandlangan panjara.",
    "yellow", comp(*grill), inst=[Location(), W("XY").plane.location if False else Location()][:1], tag="cover")
grill2 = cq.Compound.makeCompound(comp(*grill).Solids()).mirror("YZ")
add("panjara2", "Lyuk himoya panjarasi",
    "Lyuk ochiq bo'lganda ham odam qo'li yoki oyog'i qozon ichiga tushmasligi uchun payvandlangan panjara.",
    "yellow", grill2, tag="cover")
ls = comp(box(600, 680, -236, -196, 708, 760), cylY(9, -250, -236, x=640, z=735))
add("lyuk_kalit", "Lyuk xavfsizlik kaliti", "Lyuk holatini nazorat qiladigan chegaraviy kalit. Lyuk ochiq "
    "bo'lsa boshqaruv tizimi motorlarni ishga tushirmaydi.", "red", ls,
    inst=[Location(), T(-1280, 0, 0)], tag="cover")

# ================================================================= BO'SHATISH QOPQOG'I
print("bo'shatish qopqog'i")
HA = 31.0
gate = sector(GR - 12, GR, 90 - HA, 90 + HA, -627, 627)
for sy in (-1, 1):
    gate = gate.union(sector(0, GR - 1, 90 - HA, 90 + HA, min(sy * 615, sy * 627), max(sy * 615, sy * 627)))
    gate = gate.union(cylY(62, min(sy * 590, sy * 640), max(sy * 590, sy * 640)))
for y in (-210, 210):
    gate = gate.union(sector(250, GR - 6, 90 - HA, 90 + HA, y - 5, y + 5))
for a in (90 - HA + 2, 90 + HA - 2):
    gate = gate.union(box(GR - 60, GR - 6, -620, 620, -6, 6).rotate((0, 0, 0), (0, 1, 0), -a))
gate = gate.union(cylY(35, 600, 800)).union(cylY(35, -830, -600))
add("zatvor", "Sektor bo'shatish qopqog'i",
    "Silindrik sektor shaklidagi qopqoq, o'qi vallarga parallel. 65 gradus burilib, 1200 x 392 mm og'izni "
    "to'liq ochadi. Yuzasi Hardox 500 (12 mm). Uch holatli: yopiq / yarim ochiq / to'liq ochiq.",
    "liner", gate, group="gate", ang=0.12)
lever = (box(0, 222, -775, -755, -34, 34).union(cylY(70, -775, -755)).union(cylY(34, -775, -755, x=220))
         .union(cylY(16, -792, -738, x=220))).rotate((0, 0, 0), (0, 1, 0), 60)
add("richag", "Qopqoq richagi",
    "Gidrosilindr kuchini qopqoq o'qiga uzatadigan richag, yelkasi 220 mm.", "dark", lever, group="gate")
hl = (box(0, 350, 770, 784, -22, 22).union(cylY(55, 766, 788)).union(cylY(16, 760, 880, x=340))
      .union(cylY(60, 745, 757).cut(box(-100, 0, 740, 760, -100, 100)))).rotate((0, 0, 0), (0, 1, 0), -215)
add("qol_richag", "Qo'lda ochish richagi",
    "Elektr yoki gidravlika uzilganda qopqoqni qo'lda ochish uchun avariya richagi. Yonidagi kulachok "
    "chegaraviy kalitlarni bosadi.", "red", hl, group="gate")

for sy, pid in ((1, "a"), (-1, "b")):
    y0, y1 = (690, 705) if sy > 0 else (-705, -690)
    bp = (box(-130, 130, y0, y1, -1090, -440).cut(cylY(RS + 5, y0 - 5, y1 + 5, x=-C))
          .cut(cylY(RS + 5, y0 - 5, y1 + 5, x=C)).cut(cylY(37, y0 - 5, y1 + 5, z=GZ)))
    h0, h1 = (705, 745) if sy > 0 else (-745, -705)
    bh = cylY(92, h0, h1, z=GZ).cut(cylY(36, h0 - 5, h1 + 5, z=GZ))
    add("zatvor_tayanch_" + pid, "Qopqoq podshipnik tayanchi",
        "Qopqoq o'qining sirpanish podshipnigi (bronza vtulka) o'rnatilgan osma tayanch. "
        "Moylash nuqtasi markaziy tizimga ulangan.", "dark", comp(bp, bh))
lsw = comp(box(-150, -90, 708, 760, -960, -900), box(90, 150, 708, 760, -1080, -1020))
add("zatvor_kalit", "Qopqoq holati kalitlari",
    "Ikki induktiv datchik: 'yopiq' va 'ochiq' holatini boshqaruv tizimiga bildiradi. "
    "Qopqoq yopilmaguncha yangi yuklash boshlanmaydi.", "red", lsw)

# gidrosilindr (lokal: orqa sharnir (0,0,0), +X bo'ylab)
cb = comp(cylX(46, 40, 520), cylX(52, 500, 532), cylX(52, 30, 60), cylY(30, -24, 24),
          box(-10, 40, -22, 22, -30, 30), cylZ(9, 40, 62, x=90), cylZ(9, 40, 62, x=480))
add("gs_korpus", "Gidrosilindr korpusi",
    "Ikki tomonlama ta'sirli gidrosilindr D80/45, yo'li 254 mm, ish bosimi 160 bar. "
    "Qopqoqni 3-4 soniyada ochadi.", "hydr", cb, group="cylbody", ang=0.3)
cr = comp(cylX(22, -500, -30), cylY(32, -22, 22), box(-45, 0, -20, 20, -22, 22))
add("gs_shtok", "Gidrosilindr shtoki", "Xromlangan shtok, D45 mm.", "chrome", cr, group="cylrod", ang=0.3)
ab = (profY([(700, -450), (900, -350), (870, -1150), (780, -1150)], -800, -787)
      .cut(cylY(RS + 5, -810, -780, x=C)).union(cylY(17, -800, -730, x=820, z=-1100)))
add("gs_kronshteyn", "Gidrosilindr kronshteyni", "Silindrning orqa sharniri mahkamlanadigan osma kronshteyn.",
    "dark", ab)

# gidrostansiya
hpu = comp(box(600, 900, -1215, -945, 300, 560), box(580, 920, -1225, YO * -1, 285, 300),
           box(590, 610, -1215, -YO, 120, 300).cut(box(580, 620, -1215, -990, 110, 240)),
           box(890, 910, -1215, -YO, 120, 300).cut(box(880, 920, -1215, -990, 110, 240)))
add("gs_bak", "Gidrostansiya baki (40 L)",
    "Qopqoq yuritmasining moy baki, sath ko'rsatkichi va nafas filtri bilan. Torets devoridagi "
    "kronshteynga o'rnatilgan.", "gearblue", hpu)
hpm = comp(cylZ(85, 560, 800, x=690, y=-1080), cylZ(92, 800, 840, x=690, y=-1080),
           box(780, 880, -1140, -1020, 560, 650), cylY(8, -1020, -990, x=830, z=620),
           cylY(30, -1005, -990, x=830, z=620), cylZ(14, 560, 600, x=850, y=-1180), cylZ(22, 600, 615, x=850, y=-1180))
add("gs_nasos", "Gidrostansiya motor-nasosi 2,2 kVt",
    "Tishli nasos va 2,2 kVt motor, elektromagnit taqsimlagich, saqlash klapani va manometr. "
    "Gidroakkumulyator elektr uzilganda ham qopqoqni bir marta ochishga yetadi.", "motor", hpm)
hp_hand = comp(box(640, 660, -1235, -1215, 380, 700), cylY(14, -1300, -1215, x=650, z=690),
               box(630, 670, -1225, -1212, 360, 420))
add("gs_qolnasos", "Qo'l nasosi", "Avariya holatida qopqoqni qo'lda ochish uchun gidravlik qo'l nasosi.",
    "red", hp_hand)
hoses = comp(pipe([(800, -945, 620), (800, -960, 620), (935, -960, 620), (935, -960, -1060), (935, -775, -1060),
                   (890, -775, -1060)], 11),
             pipe([(850, -945, 600), (850, -975, 600), (965, -975, 600), (965, -975, -1100), (965, -745, -1100),
                   (890, -745, -1100)], 11))
add("gs_shlang", "Yuqori bosim shlanglari",
    "Gidrostansiyadan silindrga boradigan 2 ta yuqori bosim shlangi (ochish va yopish liniyalari).",
    "hydr", hoses)

# markaziy moylash
lube = comp(box(-80, 80, -1000, -YO, 250, 330), cylZ(62, 330, 520, y=-985), cylZ(66, 520, 532, y=-985),
            box(-70, 70, -1010, -1000, 262, 318))
add("moy_nasos", "Markaziy moylash nasosi",
    "Avtomatik konsistent moy nasosi, 4 chiqishli. Mikser ishlayotganda 4 ta val zichlagichiga "
    "uzluksiz moy haydaydi — zichlagichlar umrini belgilaydigan asosiy tizim. Bak sath datchigi bilan.",
    "yellow", lube)
ll = []
for sx in (-1, 1):
    ll.append(pipe([(sx * 40, -960, 290), (sx * C, -960, 290), (sx * C, -960, 150)], 5))
    ll.append(pipe([(sx * 60, -975, 300), (sx * 1090, -975, 300), (sx * 1090, 965, 300), (sx * C, 965, 300),
                    (sx * C, 965, 150)], 5))
add("moy_quvur", "Moylash quvurchalari",
    "Nasosdan har bir zichlagichga boradigan 8 mm mis quvurchalar.", "copper", comp(*ll), ang=0.6)

# boshqaruv qutisi, tablichka
jb = comp(box(1120, 1270, -160, 160, 400, 680), box(1270, 1276, -150, 150, 410, 670),
          cylX(24, 1276, 1296, y=90, z=600), cylX(14, 1276, 1290, y=-60, z=600), cylX(14, 1276, 1290, y=-110, z=600))
add("klemma_quti", "Klemma va boshqaruv qutisi",
    "Datchiklar, kalitlar va klapanlar simlari yig'iladigan quti. Old tomonida avariya to'xtatish tugmasi; "
    "ta'mirlash paytida mahalliy boshqaruv shu yerdan.", "dark", jb)
add("avariya", "Avariya to'xtatish tugmasi", "Qizil qo'ziqorin tugma: bosilsa ikkala motor va gidrostansiya "
    "darhol o'chadi.", "red", cylX(26, 1290, 1312, y=90, z=600))
add("tablichka", "Zavod tablichkasi",
    "Turi: ikki valli majburiy qorishtirgich 3000/2000. Zichlangan beton 2,0 m3, quvvat 2 x 37 kVt, "
    "massa ~7400 kg.", "silver", box(-1063, -1060, -170, 170, 420, 620))

# bo'shatish voronkasi
hood = (box(-425, 425, -655, 655, -1050, -700).cut(box(-420, 420, -650, 650, -1060, -690)))
for sy in (-1, 1):
    hood = hood.cut(cylY(45, min(sy * 640, sy * 670), max(sy * 640, sy * 670), z=GZ))
fun_o = W("XY").rect(850, 1310).workplane(offset=-400).rect(510, 510).loft()
fun_i = W("XY").rect(840, 1300).workplane(offset=-400).rect(500, 500).loft()
hood = hood.union(fun_o.cut(fun_i).translate((0, 0, -1050)))
hood = hood.union(box(-445, 445, -675, 675, -706, -700).cut(box(-420, 420, -650, 650, -710, -690)))
add("voronka", "Bo'shatish voronkasi",
    "Qopqoq ostidagi yopiq g'ilof va voronka: betonni avtomikser bo'g'ziga yo'naltiradi, sachrashni to'sadi. "
    "Ichi yeyilishga chidamli rezina bilan qoplangan.", "body", hood, tag="guard")
sock = box(-262, 262, -262, 262, -1750, -1440).cut(box(-250, 250, -250, 250, -1760, -1430))
add("yeng", "Rezina bo'shatish yengi",
    "Voronka uchidagi qalin rezina yeng. Avtomikser bo'g'ziga tegsa ham shikastlanmaydi.", "rubber", sock,
    tag="guard")

# ================================================================= EKSPORT
print("eksport")
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.dirname(here)
os.makedirs(os.path.join(root, "out"), exist_ok=True)
os.makedirs(os.path.join(root, "viewer"), exist_ok=True)


def mat34(loc):
    t = loc.wrapped.Transformation()
    return [round(t.Value(i, j), 5) for i in (1, 2, 3) for j in (1, 2, 3, 4)]


blob = bytearray()
meta = {"groups": groups, "parts": [], "params": dict(C=C, R=R, LH=LH, HT=HT, GZ=GZ, GR=GR, MW=MW, ML=ML, MZ=MZ, RT=RT)}
cache = {}
ntri = 0
for p in parts:
    key = id(p["shape"])
    if key not in cache:
        vs, ts = p["shape"].tessellate(p["tol"], p["ang"])
        vo = len(blob)
        blob += struct.pack("<%df" % (3 * len(vs)), *[c for v in vs for c in (v.x, v.y, v.z)])
        io = len(blob)
        blob += struct.pack("<%dI" % (3 * len(ts)), *[i for t in ts for i in t])
        cache[key] = (vo, len(vs), io, len(ts))
    vo, nv, io, nt = cache[key]
    ntri += nt * len(p["inst"])
    meta["parts"].append(dict(id=p["id"], name=p["name"], desc=p["desc"], mat=p["mat"], group=p["group"],
                              tag=p["tag"], v=[vo, nv], i=[io, nt], inst=[mat34(l) for l in p["inst"]]))
import base64
meta["bin"] = base64.b64encode(bytes(blob)).decode("ascii")   # to'rlar JSON ichida (base64)
with open(os.path.join(root, "viewer", "mixer.json"), "w") as f:
    json.dump(meta, f, ensure_ascii=False, separators=(",", ":"))
print("to'r: %.2f MB, %d uchburchak, %d detal turi" % (len(blob) / 1e6, ntri, len(parts)))

if "--nostep" not in sys.argv:
    COL = dict(body=(0.85, 0.84, 0.8), dark=(0.2, 0.22, 0.24), liner=(0.35, 0.35, 0.36), nihard=(0.3, 0.3, 0.32),
               steel=(0.6, 0.62, 0.65), castarm=(0.25, 0.25, 0.27), gear=(0.45, 0.47, 0.5), gearblue=(0.1, 0.3, 0.55),
               motor=(0.1, 0.3, 0.55), yellow=(0.95, 0.75, 0.1), rubber=(0.07, 0.07, 0.07), chrome=(0.9, 0.9, 0.92),
               hydr=(0.08, 0.08, 0.09), water=(0.15, 0.4, 0.7), fabric=(0.9, 0.9, 0.85), red=(0.8, 0.1, 0.1),
               silver=(0.8, 0.8, 0.8), copper=(0.7, 0.45, 0.25))
    assy = cq.Assembly(name="beton_qozon_2m3")
    for p in parts:
        gv = Location(Vector(*groups[p["group"]]["pivot"]))
        extra = Location()
        if p["group"] == "cylbody":
            extra = RY(186.5)
        if p["group"] == "cylrod":
            gv = Location(Vector(110, -765, -1180.5))
            extra = RY(186.5)
        for k, l in enumerate(p["inst"]):
            assy.add(p["shape"], name="%s_%d" % (p["id"], k), loc=gv * extra * l, color=cq.Color(*COL[p["mat"]]))
    assy.save(os.path.join(root, "out", "beton_qozon_2m3.step"))
    print("STEP yozildi")
