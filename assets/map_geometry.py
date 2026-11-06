"""Expands metric room rows and subtractive doors into box primitives and annotations."""
import math

DEFAULT_DOOR_END_MARGIN_FACTOR = 1.0

def point(origin, yaw, x, y):
  c, s = math.cos(yaw), math.sin(yaw)
  return [origin[0] + c*x - s*y, origin[1] + s*x + c*y]
def expand(spec):
  walls = list(spec['walls'])
  doors = list(spec.get('doors', []))
  annotations = list(spec.get('annotations', []))
  row_ids = set()
  for row in spec.get('room_rows', []):
    if row['id'] in row_ids:
      raise ValueError('Duplicate room row id')
    row_ids.add(row['id'])
    sign = 1 if row['side'] == 'left' else -1
    t = row['wall_thickness']
    cursor = 0
    for room in row['rooms']:
      values = dict(row, **room.get('overrides', {}))
      width, depth = values['room_width'], values['depth']
      if min(width, depth) <= t:
        raise ValueError('Room dimensions must exceed wall thickness')
      name = room['id']
      def add(suffix, x, y, sx, sy):
        walls.append(dict(id=name+'_'+suffix, center=point(row['origin'],row['yaw'],x,y),size=[sx,sy],yaw=row['yaw'],z_min=row['z_min'],z_max=row['z_max']))
      add('front',cursor+width/2,0,width,t)
      add('back',cursor+width/2,sign*depth,width,t)
      # Separate side segments allow adjacent rooms to have different depths.
      add('left',cursor,sign*depth/2,t,depth)
      add('right',cursor+width,sign*depth/2,t,depth)
      nav = point(row['origin'],row['yaw'],cursor+width/2,sign*depth/2)
      annotations.append(dict(id=name,kind='point',label=name,point=nav,nav_point=nav))
      doors.append(dict(id=name+'_door',room=name,wall=name+'_front',width=values['door_width'],offset=values['door_offset'],nav_point=nav))
      cursor += width
  wall_ids = [wall['id'] for wall in walls]
  if len(set(wall_ids)) != len(wall_ids):
    raise ValueError('Duplicate wall id')
  grouped = {name: [] for name in wall_ids}
  for door in doors:
    if door['wall'] not in grouped:
      raise ValueError('Door references unknown wall')
    grouped[door['wall']].append(door)
  primitives = []
  for wall in walls:
    if wall['z_max'] <= wall['z_min']:
      raise ValueError('z_max must exceed z_min')
    openings = sorted(grouped[wall['id']], key=lambda door: door['offset'])
    if not openings:
      primitives.append(dict(wall, type='box'))
      continue
    length, thickness = wall['size']
    cursor = -length/2
    for index, door in enumerate(openings):
      lo, hi = door['offset']-door['width']/2, door['offset']+door['width']/2
      margin = door.get('minimum_end_distance', thickness*DEFAULT_DOOR_END_MARGIN_FACTOR)
      if lo < -length/2 or hi > length/2:
        raise ValueError('Door extends beyond wall')
      if lo < cursor:
        raise ValueError('Doors overlap')
      if lo+length/2 < margin or length/2-hi < margin or lo-cursor < margin:
        raise ValueError('Door leaves a wall segment below minimum_end_distance')
      primitives.append(dict(wall,id=f"{wall['id']}_segment{index}",type='box',center=point(wall['center'],wall['yaw'],(cursor+lo)/2,0),size=[lo-cursor,thickness]))
      annotations.append(dict(id=door['id'],kind='door',room=door['room'],center=point(wall['center'],wall['yaw'],door['offset'],0),width=door['width'],yaw=wall['yaw'],nav_point=door['nav_point']))
      cursor = hi
    primitives.append(dict(wall,id=f"{wall['id']}_segment{len(openings)}",type='box',center=point(wall['center'],wall['yaw'],(cursor+length/2)/2,0),size=[length/2-cursor,thickness]))
  ids = [item['id'] for item in primitives+annotations]
  if len(ids) != len(set(ids)):
    raise ValueError('Duplicate primitive or annotation id')
  points = {a['id'] for a in annotations if a['kind']=='point'}
  if any(a['room'] not in points for a in annotations if a['kind']=='door'):
    raise ValueError('Door room must reference a point annotation')
  if not primitives and not spec.get('obstacles'):
    raise ValueError('Map requires at least one primitive')
  return primitives, annotations
