from types import SimpleNamespace

from opendbc.sunnypilot.car.ford.longitudinal_ext import LongitudinalExt


class FakeSubMaster(dict):
  def __init__(self, lead):
    super().__init__(radarState=SimpleNamespace(leadOne=lead))
    self.valid = {"radarState": True}


def make_controller():
  controller = LongitudinalExt(None, None)
  controller.sm = FakeSubMaster(SimpleNamespace(
    status=1,
    dRel=50.0,
    vRel=-1.0,
    vLead=25.0,
  ))

  cc = SimpleNamespace(longActive=True)
  cs = SimpleNamespace(out=SimpleNamespace(
    vEgo=27.0,
    gasPressed=False,
    brakePressed=False,
  ))
  return controller, cc, cs


def update(controller, cc, cs, accel):
  # Seed bp_accel_last in each test before the first call so the normal
  # following-rate limiter does not hide the hysteresis threshold under test.
  return controller.update(
    cc,
    cs,
    op_accel=accel,
    op_gas=accel,
    accel_due_to_pitch=0.0,
    v_ego_mph=60.0,
    stopping=False,
    target_speed=100.0,
  )


def test_bp_brake_hysteresis_persists_until_release_threshold():
  controller, cc, cs = make_controller()
  controller.bp_accel_last = -0.15

  engaged = update(controller, cc, cs, -0.15)
  assert engaged.bp_long_used
  assert engaged.brake_actuate

  # This is the failure seen in route logs: the request moves back above
  # -0.14 but is still well below the intended -0.06 release threshold.
  held = update(controller, cc, cs, -0.10)
  assert held.brake_actuate

  released = update(controller, cc, cs, -0.05)
  assert not released.brake_actuate


def test_bp_precharge_hysteresis_persists_until_release_threshold():
  controller, cc, cs = make_controller()
  controller.bp_accel_last = -0.13

  engaged = update(controller, cc, cs, -0.13)
  assert engaged.bp_long_used
  assert engaged.precharge_actuate
  assert not engaged.brake_actuate

  held = update(controller, cc, cs, -0.10)
  assert held.precharge_actuate

  released = update(controller, cc, cs, -0.05)
  assert not released.precharge_actuate


def test_bp_latches_sync_when_bp_long_relinquishes_control():
  controller, cc, cs = make_controller()
  controller.bp_accel_last = -0.15

  engaged = update(controller, cc, cs, -0.15)
  assert engaged.brake_actuate
  assert controller.bp_brake_actuate_last
  assert controller.bp_precharge_actuate_last

  # Simulate a driver pedal override while the upstream request has already
  # recovered above both release thresholds. BP should hand control back and
  # clear/synchronize its latches instead of resurrecting stale brake state.
  cs.out.gasPressed = True
  relinquished = update(controller, cc, cs, 0.0)
  assert not relinquished.bp_long_used
  assert not controller.bp_brake_actuate_last
  assert not controller.bp_precharge_actuate_last
