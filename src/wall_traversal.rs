//! Source-backed held/retry state with explicit LAB transport/service bindings.
use crate::wall_query::{Query,Contact};
use serde::{Deserialize,Serialize};
use std::collections::BTreeMap;

#[derive(Clone,Copy,Debug,Default,Serialize,Deserialize,PartialEq,Eq)]
pub struct JumpActions {
    pub authoritative:bool, pub first_id:u64, pub active_bits:u64,
    pub crouch_bits:u64, pub len:u8,
}
impl JumpActions {
    pub fn valid(&self)->bool {
        let mask=if self.len==64 {u64::MAX}else if self.len<64 {(1u64<<self.len)-1}else{return false;};
        self.active_bits&!mask==0 && self.crouch_bits&!self.active_bits==0
            && self.first_id.checked_add(self.len as u64).is_some()
            && (self.authoritative || self.len==0)
    }
    pub fn consumed(&mut self) {self.first_id+=self.len as u64;self.len=0;self.active_bits=0;self.crouch_bits=0;}
}
#[derive(Clone,Copy,Debug,Default,Serialize,Deserialize,PartialEq,Eq)]
pub enum Provider { #[default] Static, Pending, Refused, Unsupported }
#[derive(Clone,Copy,Debug,Serialize,Deserialize)]
#[serde(default)]
pub struct Policy {
    pub wall_enabled:bool, pub input_enabled:bool, pub competing_move:bool,
    pub actor_admits_wall:bool, pub provider:Provider,
}
impl Default for Policy {
    fn default()->Self {Self{wall_enabled:true,input_enabled:true,competing_move:false,actor_admits_wall:true,provider:Provider::Static}}
}
#[derive(Clone,Copy,Debug,Serialize,Deserialize,PartialEq,Eq)]
pub enum Kind {Initial,Retry}
#[derive(Clone,Debug,Serialize,Deserialize)]
pub struct Ticket {pub kind:Kind,pub press_id:Option<u64>,pub crouch:bool,pub query:Query,pub pending:bool}
#[derive(Clone,Copy,Debug,Serialize,Deserialize,PartialEq,Eq)]
pub enum OutcomeKind {WallHandled,Ordinary,InitialFallback,WallLaunch}
#[derive(Clone,Copy,Debug,Serialize,Deserialize,PartialEq,Eq)]
pub enum ResultKind {Accepted,Roll,InputDisabled, EvaluatedProperty,Budget,WallHandled,LaunchTimer,Actor,Unsupported}
#[derive(Clone,Copy,Debug,Serialize,Deserialize,PartialEq,Eq)]
pub enum ProviderStatus {Submitted,Pending,Refused,Unsupported}
#[derive(Clone,Debug,Serialize,Deserialize)]
pub struct Outcome {pub kind:OutcomeKind,pub id:Option<u64>,pub crouch:bool,pub result:ResultKind}
#[derive(Clone,Copy,Debug,Serialize,Deserialize)]
pub struct Disposition {pub handled:bool,pub crouch:bool}
#[derive(Clone,Debug,Serialize,Deserialize)]
#[serde(default)]
pub struct State {
    pub held:bool, pub retry_timer:f32,pub ticket:Option<Ticket>,pub selected:Option<Contact>,
    pub dispositions:BTreeMap<u64,Disposition>,pub next_id:Option<u64>,pub authoritative:Option<bool>,
    pub legacy_held:bool,pub legacy_id:u64,pub outcomes:Vec<Outcome>,pub error:Option<String>,
    pub submitted:u64,pub launches:u64,pub published_vector:Option<glam::Vec3>,pub last_provider:Option<ProviderStatus>,
}
impl Default for State {
    fn default()->Self {Self{held:false,retry_timer:f32::MAX,ticket:None,selected:None,
        dispositions:BTreeMap::new(),next_id:None,authoritative:None,legacy_held:false,legacy_id:0,
        outcomes:Vec::new(),error:None,submitted:0,launches:0,published_vector:None,last_provider:None}}
}
impl State {
    pub fn hard_cancel(&mut self) {
        self.held=false;self.retry_timer=f32::MAX;self.ticket=None;self.dispositions.clear();
        self.next_id=None;self.authoritative=None;self.legacy_held=false;
    }
    pub fn outcome(&mut self,kind:OutcomeKind,id:Option<u64>,crouch:bool,result:ResultKind) {
        self.outcomes.push(Outcome{kind,id,crouch,result});
    }
    pub fn valid_for(&self,world:&crate::World)->bool {
        self.retry_timer.is_finite() && self.ticket.as_ref().is_none_or(|t|t.query.valid_for(world))
        && self.selected.is_none_or(|c|c.normal.is_finite()&&c.point.is_finite()&&c.score.is_finite()&&c.pair<8)
        && self.published_vector.is_none_or(|v|v.is_finite())
        && self.dispositions.keys().all(|id|self.next_id.is_some_and(|next|*id<next))
    }
}
