//! Ordered LAB keyboard actions and separately drained, associated ordinary edges.
use bevy::input::{ButtonState,keyboard::KeyCode};
use std::collections::VecDeque;
use movement_lab::wall_traversal::JumpActions;
#[derive(Clone,Copy,Debug)]
pub struct Edge {pub id:u64,pub crouch:bool}
#[derive(Clone,Copy,Debug)]
pub struct Action {pub id:u64,pub active:bool,pub crouch:bool}
#[derive(Default)]
pub struct JumpEdges {
    ctrl:bool,c:bool,space:bool,pending:VecDeque<Edge>,actions:VecDeque<Action>,next_id:u64,
}
impl JumpEdges {
    pub fn record(&mut self,key:KeyCode,state:ButtonState,repeat:bool) {
        let pressed=state==ButtonState::Pressed;
        if pressed && repeat && key==KeyCode::Space {return;}
        match key {
            KeyCode::ControlLeft=>self.ctrl=pressed,KeyCode::KeyC=>self.c=pressed,
            KeyCode::Space=>{
                if pressed!=self.space {
                    let id=self.next_id;self.next_id=self.next_id.checked_add(1).expect("Space transition ID exhausted");
                    let crouch=pressed&&(self.ctrl||self.c);
                    self.actions.push_back(Action{id,active:pressed,crouch});
                    if pressed {self.pending.push_back(Edge{id,crouch});}
                    self.space=pressed;
                }
            },_=>{}
        }
    }
    pub fn held(&self)->bool {self.space}
    #[cfg(test)] pub fn pending_snapshot(&self)->Vec<bool> {self.pending.iter().map(|e|e.crouch).collect()}
    pub fn pending_len(&self)->usize {self.pending.len()}
    #[cfg(test)] pub fn action_snapshot(&self)->Vec<(u64,bool,bool)> {self.actions.iter().map(|a|(a.id,a.active,a.crouch)).collect()}
    pub fn action_len(&self)->usize {self.actions.len()}
    pub fn detach_since(&mut self,n:usize)->VecDeque<Edge> {self.pending.split_off(n)}
    pub fn detach_actions_since(&mut self,n:usize)->VecDeque<Action> {self.actions.split_off(n)}
    pub fn append_pending(&mut self,q:VecDeque<Edge>) {self.pending.extend(q);}
    pub fn append_actions(&mut self,q:VecDeque<Action>) {self.actions.extend(q);}
    #[cfg(test)] pub fn take(&mut self)->Option<bool> {self.pending.pop_front().map(|e|e.crouch)}
    pub fn packet(&mut self)->JumpActions {
        let first=self.actions.front().map_or(self.next_id,|a|a.id);
        let mut p=JumpActions{authoritative:true,first_id:first,..Default::default()};
        while p.len<64 {
            let Some(a)=self.actions.pop_front() else {break;};
            assert_eq!(a.id,first+p.len as u64,"noncontiguous action queue");
            if a.active {p.active_bits|=1u64<<p.len;}
            if a.crouch {p.crouch_bits|=1u64<<p.len;}
            p.len+=1;
        }
        p
    }
    pub fn take_delivered(&mut self,end:u64)->Option<Edge> {
        if self.pending.front().is_some_and(|e|e.id<end) {self.pending.pop_front()}else{None}
    }
    pub fn is_pending(&self)->bool {!self.pending.is_empty()}
    pub fn clear_pending(&mut self) {self.pending.clear();self.actions.clear();}
    pub fn reset(&mut self) {self.clear_pending();self.ctrl=false;self.c=false;self.space=false;}
}

#[cfg(test)]
mod tests {
    use super::*;
    fn press(e: &mut JumpEdges, k: KeyCode) { e.record(k, ButtonState::Pressed, false); }
    fn release(e: &mut JumpEdges, k: KeyCode) { e.record(k, ButtonState::Released, false); }
    #[test]
    fn full_chord_released_in_one_event_batch_keeps_press_posture() {
        let mut e = JumpEdges::default();
        press(&mut e, KeyCode::ControlLeft); press(&mut e, KeyCode::Space);
        release(&mut e, KeyCode::Space); release(&mut e, KeyCode::ControlLeft);
        assert_eq!(e.take(), Some(true)); assert_eq!(e.take(), None);
    }
    #[test]
    fn later_ctrl_does_not_reclassify_earlier_space() {
        let mut e = JumpEdges::default();
        press(&mut e, KeyCode::Space); press(&mut e, KeyCode::ControlLeft);
        assert_eq!(e.take(), Some(false));
    }
    #[test]
    fn multiple_edges_keep_individual_chords_and_repeats_do_not_duplicate() {
        let mut e = JumpEdges::default();
        press(&mut e, KeyCode::Space); release(&mut e, KeyCode::Space);
        press(&mut e, KeyCode::KeyC); press(&mut e, KeyCode::Space);
        e.record(KeyCode::Space, ButtonState::Pressed, true);
        press(&mut e, KeyCode::Space);
        assert_eq!(e.take(), Some(false)); assert_eq!(e.take(), Some(true));
        assert_eq!(e.take(), None);
    }
    #[test]
    fn focus_reset_discards_queued_edges_and_stale_modifiers() {
        let mut e = JumpEdges::default();
        press(&mut e, KeyCode::ControlLeft); press(&mut e, KeyCode::Space);
        e.reset(); assert_eq!(e.take(), None);
        press(&mut e, KeyCode::Space); assert_eq!(e.take(), Some(false));
    }
}
