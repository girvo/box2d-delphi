unit UContinuousTest;

interface
{$I ..\..\Physics2D\Physics2D.inc}

uses
   UMain, UPhysics2DTypes, UPhysics2D, SysUtils;

type
   TContinuousTest = class(TTester)
   private
      procedure Launch;
   public
      m_body: Tb2Body;
      m_angularVelocity: PhysicsFloat;

      constructor Create; override;
      procedure Step(var settings: TSettings; timeStep: PhysicsFloat); override;
   end;

implementation

{ TContinuousTest }

constructor TContinuousTest.Create;
var
   bd: Tb2BodyDef;
   body: Tb2Body;
   edge: Tb2EdgeShape;
   shape: Tb2PolygonShape;
begin
   inherited;
   begin
      bd := Tb2BodyDef.Create;
      SetValue(bd.position, 0.0, 0.0);
      body := m_world.CreateBody(bd);

      edge := Tb2EdgeShape.Create;
      edge.SetVertices(MakeVector(-10.0, 0.0), MakeVector(10.0, 0.0));
      body.CreateFixture(edge, 0, True, False);

      shape := Tb2PolygonShape.Create;
      shape.SetAsBox(0.2, 1.0, MakeVector(0.5, 1.0), 0.0);
      body.CreateFixture(shape, 0.0);
   end;

   begin
      bd := Tb2BodyDef.Create;
      bd.bodyType := b2_dynamicBody;
      SetValue(bd.position, 0.0, 20.0);
      //bd.angle := 0.1;

      shape := Tb2PolygonShape.Create;
      shape.SetAsBox(2.0, 0.1);

      m_body := m_world.CreateBody(bd);
      m_body.CreateFixture(shape, 1.0);

      m_angularVelocity := RandomFloat(-50, 50);
      //m_angularVelocity = 46.661274f;
      m_body.SetLinearVelocity(MakeVector(0.0, -100.0));
      m_body.SetAngularVelocity(m_angularVelocity);
   end;

   b2_gjkCalls := 0; b2_gjkIters := 0; b2_gjkMaxIters := 0;
   b2_toiCalls := 0; b2_toiIters := 0;
   b2_toiRootIters := 0; b2_toiMaxRootIters := 0;
   b2_toiTime := 0.0; b2_toiMaxTime := 0.0;
end;

procedure TContinuousTest.Step(var settings: TSettings; timeStep: PhysicsFloat);
begin
   inherited;

   if b2_gjkCalls > 0 then
      DrawText(Format('gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d',
         [b2_gjkCalls, b2_gjkIters / b2_gjkCalls, b2_gjkMaxIters]));

   if b2_toiCalls > 0 then
   begin
      DrawText(Format('toi calls = %d, ave [max] toi iters = %3.1f [%d]',
                [b2_toiCalls, b2_toiIters / b2_toiCalls, b2_toiMaxRootIters]));

      DrawText(Format('ave [max] toi root iters = %3.1f [%d]',
        [b2_toiRootIters / b2_toiCalls, b2_toiMaxRootIters]));

      {$IFDEF COMPUTE_PHYSICS_TIME}
      DrawText(Format('ave [max] toi time = %.1f [%.1f] (microseconds)',
        [1000.0 * 1000.0 * b2_toiTime / b2_toiCalls, 1000.0 * 1000.0 * b2_toiMaxTime]));
      {$ENDIF}
   end;

   if m_stepCount mod 60 = 0 then
      //Launch;
end;

procedure TContinuousTest.Launch;
begin
   b2_gjkCalls := 0; b2_gjkIters := 0; b2_gjkMaxIters := 0;
   b2_toiCalls := 0; b2_toiIters := 0;
   b2_toiRootIters := 0; b2_toiMaxRootIters := 0;
   b2_toiTime := 0.0; b2_toiMaxTime := 0.0;

   m_body.SetTransform(MakeVector(0.0, 20.0), 0.0);
   m_angularVelocity := RandomFloat(-50, 50);
   m_body.SetLinearVelocity(MakeVector(0.0, -100.0));
   m_body.SetAngularVelocity(m_angularVelocity);
end;

initialization
   RegisterTestEntry('Continuous Test', TContinuousTest);
end.

