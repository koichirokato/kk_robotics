#[derive(Debug, Clone, serde::Serialize)]
pub struct CmdVel {
    pub linear: f32,
    pub angular: f32,
}
