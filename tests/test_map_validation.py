import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml
from assets.build import ROOT, compile_map
from assets.build_robot import compile_robot
from tools.validate_map import clear_width, rectangle, radius_for, validate, validate_geometry

SOURCE=ROOT/'spec/maps/synthetic_ward.yaml'

class MapValidationTests(unittest.TestCase):
  def setUp(self):
    self.robot=compile_robot(ROOT/'spec/robots/a100.yaml')
    self.config=dict(start='a',targets=['b'],bounds=dict(center=[0,0],size=[4,4],yaw=0))
  def fixture(self,width=1):
    def box(name,x,y,sx,sy,lo=0,hi=2):return dict(id=name,type='box',center=[x,y],size=[sx,sy],yaw=0,z_min=lo,z_max=hi)
    walls=[box('left',0,-1-width/4,.2,2-width/2),box('right',0,1+width/4,.2,2-width/2)]
    return dict(primitives=walls,annotations=[dict(id=name,kind='point',nav_point=p) for name,p in [('a',[-1,0]),('b',[1,0])]])
  def test_synthetic_validation_block_and_clear_widths(self):
    result=validate(SOURCE)
    self.assertTrue(result['ok'],result)
    self.assertEqual(result['radius'],.27)
    self.assertAlmostEqual(result['required_width'],.64)
    self.assertEqual(len(result['clear_widths']),7)
    for width in result['clear_widths'].values():self.assertAlmostEqual(width,1.2)
    self.assertEqual(self.robot['body'],dict(z_min=0,z_max=.25))
  def test_narrow_passage_fails_and_wide_passes(self):
    self.assertTrue(validate_geometry(self.fixture(1),self.robot,self.config)['ok'])
    self.assertFalse(validate_geometry(self.fixture(.4),self.robot,self.config)['ok'])
  def test_nav_point_in_wall_fails(self):
    data=self.fixture();data['annotations'][1]['nav_point']=[0,1]
    result=validate_geometry(data,self.robot,self.config)
    self.assertFalse(result['ok']);self.assertTrue(any('not in free space' in e for e in result['errors']))
  def test_height_filter_both_directions_and_zone_exclusion(self):
    data=self.fixture()
    blocker=dict(id='rail',center=[0,0],size=[.2,1],yaw=0,z_min=.7,z_max=.9)
    data['primitives'].append(blocker)
    data['annotations'].append(dict(id='zone',kind='zone',center=[0,1],size=[1,1],yaw=0))
    self.assertTrue(validate_geometry(data,self.robot,self.config)['ok'])
    blocker.update(z_min=0,z_max=.2)
    self.assertFalse(validate_geometry(data,self.robot,self.config)['ok'])
  def test_clear_width_intrusion_and_front_blocker(self):
    door=dict(center=[0,0],width=1.2,yaw=0)
    obstacle=dict(center=[.45,0],size=[.3,.4],yaw=0)
    self.assertAlmostEqual(clear_width(door,rectangle(obstacle)),.9)
    obstacle['center']=[0,0];obstacle['size']=[.2,.4]
    self.assertAlmostEqual(clear_width(door,rectangle(obstacle)),.5)
    obstacle['center']=[0,.8];obstacle['size']=[2,.2]
    self.assertAlmostEqual(clear_width(door,rectangle(obstacle)),1.2)
  def test_non_circular_circumscribed_radius(self):
    self.assertAlmostEqual(radius_for(dict(type='rect',size=[.6,.8])),.5)
    self.assertAlmostEqual(radius_for(dict(type='polygon',vertices=[[.3,.4],[-.3,.4],[0,-.4]])),.5)
  def test_no_reference_dependency_or_shapely_on_compile_path(self):
    result=subprocess.run([sys.executable,'-c',"import sys; from pathlib import Path; from assets.build import compile_map; compile_map(Path('spec/maps/synthetic_ward.yaml')); assert not any(x.startswith('shapely') for x in sys.modules)"],cwd=ROOT,capture_output=True,text=True)
    self.assertEqual(result.returncode,0,result.stderr)
    compiled=compile_map(SOURCE)
    self.assertNotIn('reference',compiled);self.assertNotIn('validation',compiled)
    self.assertNotIn('catalog',compiled)
    self.assertEqual(compiled,json.loads((ROOT/'sim/generated/synthetic_ward.json').read_text()))
  def test_new_block_validation_and_catalog_errors(self):
    source=yaml.safe_load(SOURCE.read_text())
    source['catalog']=str(ROOT/'spec/obstacles/catalog.yaml')
    with tempfile.TemporaryDirectory() as tmp:
      path=Path(tmp)/'map.yaml'
      for key,value in [('reference',{'unexpected':True}),('validation',{'clearance':-.1}),('spawn_regions',[{'kind':'zone'}]),('obstacles',[{'id':'bad'}])]:
        bad=copy.deepcopy(source);bad[key]=value;path.write_text(yaml.safe_dump(bad))
        with self.assertRaises(ValueError):compile_map(path)
      for field in ['center','size','yaw','z_min','z_max','estimated']:
        bad=copy.deepcopy(source);del bad['obstacles'][-1]['parts'][0][field];path.write_text(yaml.safe_dump(bad))
        with self.assertRaises(ValueError):compile_map(path)
      for field,value in [('size',[0,1]),('z_max',0),('yaw',float('nan')),('extra',1)]:
        bad=copy.deepcopy(source);bad['obstacles'][-1]['parts'][0][field]=value;path.write_text(yaml.safe_dump(bad))
        with self.assertRaises(ValueError):compile_map(path)
      catalog=Path(tmp)/'catalog.yaml';catalog.write_text('version: 1\nversion: 1\n')
      source['catalog']=str(catalog);path.write_text(yaml.safe_dump(source))
      with self.assertRaisesRegex(ValueError,'Duplicate YAML key'):compile_map(path)
  def test_cli_negative_fixture_exits_nonzero(self):
    source=yaml.safe_load(SOURCE.read_text())
    source['catalog']=str(ROOT/'spec/obstacles/catalog.yaml')
    source['validation']['robot']=str(ROOT/'spec/robots/a100.yaml')
    source['room_rows'][0]['door_width']=.4
    with tempfile.TemporaryDirectory() as tmp:
      path=Path(tmp)/'narrow.yaml';path.write_text(yaml.safe_dump(source))
      result=subprocess.run([sys.executable,'-m','tools.validate_map',str(path)],cwd=ROOT,capture_output=True,text=True)
      self.assertEqual(result.returncode,1,result.stdout+result.stderr)
      self.assertIn('CONNECTIVITY_FAIL',result.stdout)
      self.assertIn('unreachable',result.stdout)
