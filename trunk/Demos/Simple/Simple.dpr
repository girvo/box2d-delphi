program Simple;

uses
  Forms,
  UPhysicsDebug in 'UPhysicsDebug.pas' {frmDebug},
  UPhysics2D in '..\..\Source\UPhysics2D.pas',
  UPhysics2DTypes in '..\..\Source\UPhysics2DTypes.pas';

{$R *.res}

begin
  Application.Initialize;
  //Application.MainFormOnTaskbar := True;
  Application.CreateForm(TfrmDebug, frmDebug);
  Application.Run;
end.
