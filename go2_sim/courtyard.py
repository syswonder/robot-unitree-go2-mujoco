# SPDX-License-Identifier: Apache-2.0
"""Procedural campus courtyard. All obstacles have real collision geometry."""
import xml.etree.ElementTree as ET

ZONES = [
    ('A  AGILITY', (0., -1.8, .12)),
    ('B  HURDLE JUMP', (4., -1.8, .12)),
    ('C  SLALOM', (4., 7.0, .12)),
    ('D  STAIRS', (-1., 6.5, .12)),
    ('E  PERFORMANCE', (-5., -1.8, .12)),
    ('F  RAMP', (-5., 6.5, .12)),
]

def build(world, asset):
    ET.SubElement(world, 'light', name='courtyard_fill', directional='true',
                  pos='0 -4 6', dir='0 1 -1', diffuse='.20 .20 .20',
                  ambient='.08 .08 .08', specular='0 0 0', castshadow='false')
    ET.SubElement(asset, 'material', name='courtyard_matte', specular='0',
                  shininess='0', reflectance='0')
    world.find("geom[@name='floor']").set('material', 'courtyard_matte')
    def geom(name, kind, pos, size, color, collision=True, **attrs):
        return ET.SubElement(world, 'geom', name=name, type=kind,
            pos=' '.join(map(str,pos)), size=' '.join(map(str,size)),
            rgba=color, material='courtyard_matte',
            **({} if collision else {'contype':'0','conaffinity':'0'}), **attrs)
    # Non-colliding color inlays mark zones without invisible steps.
    geom('motion_zone','box',(0,0,.001),(1.3,1.3,.001),'.33 .45 .52 1',False)
    geom('jump_zone','box',(4.,0,.001),(1.25,1.3,.001),'.56 .46 .30 1',False)
    geom('slalom_zone','box',(4.,5.,.001),(1.35,1.65,.001),'.32 .48 .39 1',False)
    geom('dance_zone','box',(-5.,0,.001),(1.4,1.4,.001),'.47 .37 .53 1',False)
    geom('stairs_zone','box',(-1.,5.,.001),(1.9,1.,.001),'.37 .44 .53 1',False)
    geom('ramp_zone','box',(-5.,5.,.001),(1.15,1.25,.001),'.49 .49 .32 1',False)
    for x in range(-7,8):
        geom(f'paving_x{x}','box',(x,2.,.0005),(.008,6.,.0005),'.27 .30 .32 1',False)
    for y in range(-4,9):
        geom(f'paving_y{y}','box',(0,y,.0005),(7.5,.008,.0005),'.27 .30 .32 1',False)
    # 4 cm hurdle: geometry remains fixed throughout a run.
    geom('jump_hurdle','box',(4.,0,.02),(.02,.70,.02),'.91 .38 .16 1')
    for y in (-.75,.75):
        geom(f'hurdle_marker{y}','cylinder',(4.,y,.15),(.03,.15),'.20 .30 .43 1')
    for i,(x,y) in enumerate(((3.6,4.),(4.4,5.),(3.6,6.))):
        geom(f'slalom_{i}','cylinder',(x,y,.28),(.12,.28),'.18 .56 .61 1')
        geom(f'slalom_cap_{i}','cylinder',(x,y,.57),(.125,.018),'.94 .68 .20 1')
    # A symmetric stair bridge: three ascending and three descending treads.
    for i,h in enumerate((.05,.10,.15,.15,.15,.10,.05)):
        geom(f'step_{i}','box',(-2.2+i*.4,5.,h/2),(.20,.65,h/2),'.48 .61 .72 1')
    # Two touching shallow slopes form a ramp bridge; high point is 16 cm.
    for i,sign in enumerate((-1,1)):
        geom(f'ramp_{i}','box',(-5.,5.+sign*.5,.065),(.65,.505,.015),
             '.63 .65 .41 1',euler=f'{-sign*.15} 0 0')
    # Cafe facade, windows and entrance are scenery, placed beyond test lanes.
    geom('cafe','box',(.5,8.8,1.3),(6.5,.22,1.3),'.86 .89 .90 1')
    geom('cafe_roof','box',(.5,8.55,2.65),(6.7,.65,.10),'.24 .35 .45 1')
    for i,x in enumerate((-3.,-1.5,1.5,3.)):
        geom(f'window_{i}','box',(x,8.56,1.45),(.58,.025,.62),'.27 .48 .60 1')
    geom('door','box',(0,8.555,.95),(.45,.03,.95),'.22 .32 .42 1')
    # Benches and planters: static collision obstacles at the boundary.
    for i,x in enumerate((-3.,3.8)):
        geom(f'bench_seat{i}','box',(x,-2.7,.42),(.7,.25,.045),'.55 .35 .20 1')
        geom(f'bench_back{i}','box',(x,-2.94,.68),(.7,.035,.22),'.55 .35 .20 1')
        for j,dx in enumerate((-.5,.5)):
            geom(f'bench_leg{i}{j}','box',(x+dx,-2.7,.2),(.04,.20,.20),'.20 .27 .30 1')
    for i,(x,y) in enumerate(((-7,-3.8),(7,-3.8),(-7,7.8),(7,7.8))):
        geom(f'planter{i}','box',(x,y,.22),(.38,.38,.22),'.57 .63 .63 1')
        geom(f'tree_trunk{i}','cylinder',(x,y,.80),(.07,.55),'.38 .25 .16 1')
        geom(f'tree_crown{i}','ellipsoid',(x,y,1.6),(.60,.55,.70),'.20 .43 .28 1')
    for i,x in enumerate((-7.,7.)):
        geom(f'lamp_post{i}','cylinder',(x,0,1.3),(.035,1.3),'.23 .28 .34 1')
        geom(f'lamp_head{i}','box',(x,0,2.6),(.18,.18,.04),'.95 .91 .70 1')
