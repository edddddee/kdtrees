#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::convert::From;
use std::fmt::Debug;
use std::num::NonZero;
use std::ops::{Add, Div, Mul, Range, Sub};

// TODO: Add errors to methods that can fail
// TODO: Add method to query the n nearest points
// TODO: Add method to query all points within a distance d from a point

pub const MAX_DEPTH: usize = 32;

#[derive(Clone, Debug)]
pub struct KdTreeBase<T, Item, const K: usize, Idx = usize>
where
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    pub root: Node<T, Item, K, Idx>,
    pub count: usize,
    pub max_depth: Option<usize>,
    #[allow(dead_code)]
    capacity: NonZero<usize>,
}

#[derive(Clone, Debug)]
pub enum Node<T, Item, const K: usize, Idx = usize>
where
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    Leaf {
        ranges: [Range<T>; K],
        items: Vec<Item>,
        ids: Vec<Idx>,
        capacity: NonZero<usize>,
        depth: usize,
        max_depth: Option<usize>,
    },
    Parent {
        ranges: [Range<T>; K],
        children: Box<[Node<T, Item, K, Idx>; 1 << K]>,
        #[allow(dead_code)]
        capacity: NonZero<usize>,
        depth: usize,
        max_depth: Option<usize>,
    },
}

pub trait AsCoordinates<T, const K: usize> {
    fn coordinates(self) -> [T; K];
}
impl<S, T, const K: usize> AsCoordinates<T, K> for S
where
    T: Copy,
    S: Into<[T; K]>,
{
    fn coordinates(self) -> [T; K] {
        self.into()
    }
}

pub trait Zero {
    fn zero() -> Self;
}

impl Zero for f64 {
    #[inline]
    fn zero() -> Self {
        0.0
    }
}

impl Zero for f32 {
    #[inline]
    fn zero() -> Self {
        0.0
    }
}

pub trait AlmostZero: Sized + Zero + Sub<Output = Self> + PartialOrd {
    fn almost_zero(&self) -> bool;
}

impl AlmostZero for f64 {
    fn almost_zero(&self) -> bool {
        let diff = self - Self::zero();
        -f64::EPSILON <= diff && diff <= f64::EPSILON
    }
}

impl AlmostZero for f32 {
    fn almost_zero(&self) -> bool {
        let diff = self - Self::zero();
        -f32::EPSILON <= diff && diff <= f32::EPSILON
    }
}

pub trait Two {
    fn two() -> Self;
}

impl<T> Two for T
where
    T: From<f32>,
{
    #[inline]
    fn two() -> Self {
        2.0.into()
    }
}

pub trait Halveable:
    Sized
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Div<Output = Self>
    + Mul<Output = Self>
    + Two
    + AlmostZero
    + std::fmt::Debug // TODO: Remove
{
}

impl<T> Halveable for T where
    T: Sized
        + PartialOrd
        + Add<Output = Self>
        + Sub<Output = Self>
        + Div<Output = Self>
        + Mul<Output = Self>
        + Two
        + AlmostZero
        + std::fmt::Debug // TODO: Remove
{
}

pub trait MidPoint<T> {
    fn mid_point(&self) -> T;
}

pub trait Bounds<Rhs = Self> {
    fn covers(&self, rhs: &Rhs) -> bool;
}

impl<T, const K: usize> Bounds<[(T, T); K]> for [Range<T>; K]
where
    T: PartialOrd,
{
    fn covers(&self, rhs: &[(T, T); K]) -> bool {
        !self.iter().zip(rhs.iter()).any(|(range, bounds)| {
            bounds.1 < range.start || bounds.0 > range.end
        })
    }
}

impl<T> MidPoint<T> for Range<T>
where
    T: Halveable + Clone,
{
    fn mid_point(&self) -> T {
        (self.end.clone() + self.start.clone()) / T::two()
    }
}

impl<T, Item, Idx, const K: usize> Node<T, Item, K, Idx>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    fn new(
        ranges: [Range<T>; K],
        capacity: NonZero<usize>,
        depth: usize,
        max_depth: Option<usize>,
    ) -> Self {
        Self::Leaf {
            ranges,
            capacity,
            items: Vec::with_capacity(1 * capacity.get()),
            ids: Vec::with_capacity(1 * capacity.get()),
            depth,
            max_depth,
        }
    }

    #[inline]
    fn ranges(&self) -> &[Range<T>; K] {
        match self {
            Self::Leaf { ranges, .. } => ranges,
            Self::Parent { ranges, .. } => ranges,
        }
    }

    // If [a, b) is the parent node's range along a certain axis,
    // then the child's range along that axis will be either
    // [a, c) or [c, b), where c = (b-a) / 2 is the midpoint.
    // As such, since there are K axes, a split will result in
    // 2^K distinct children. We can encode the unique
    // configuration of a particular child by using the 2^K
    // binary numbers from 0 to 2^K - 1. If the i:th bit is 1,
    // then the range along the i:th axis is [c, b), otherwise
    // the range is [a, c).
    fn split(&mut self) {
        let self_owned = unsafe { std::ptr::read(self) };
        match self_owned {
            // Splitting path
            Self::Leaf {
                ranges,
                capacity,
                depth,
                max_depth,
                ..
            } => {
                let children: [Self; 1 << K] =
                    std::array::from_fn(|combination| {
                        let subranges: [Range<T>; K] =
                            std::array::from_fn(|i| {
                                let range = &ranges[i];
                                let mid_point = range.mid_point();
                                if combination & (1 << i) == 0 {
                                    range.start.clone()..mid_point
                                } else {
                                    mid_point..range.end.clone()
                                }
                            });
                        Node::new(subranges, capacity, depth + 1, max_depth)
                    });

                let new_self = Self::Parent {
                    ranges,
                    capacity,
                    children: Box::new(children),
                    depth,
                    max_depth,
                };
                unsafe {
                    std::ptr::write(self, new_self);
                };
            }
            other => {
                unsafe {
                    std::ptr::write(self, other);
                };
            }
        }
    }
}

impl<T, Idx, const K: usize> Node<T, [T; K], K, Idx>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    fn insert(&mut self, point: [T; K], id: Idx) {
        match &mut *self {
            Self::Leaf {
                items,
                ids,
                capacity,
                depth,
                max_depth,
                ranges,
                ..
            } => {
                //println!("depth: {}", depth);
                items.push(point);
                ids.push(id);
                let mut max_depth_reached = false;
                if let Some(max_depth) = max_depth {
                    if depth >= max_depth {
                        max_depth_reached = true;
                    }
                }
                // Test if node has reached its capacity
                let test1 = items.len() > capacity.get();
                // Test if node has reached the maximum depth of the tree
                // (if it exists)
                let test2 = !max_depth_reached;
                // Test if any ranged has collapsed, for example due to range
                // being smaler than machine epsilon.
                let test3 = !ranges.iter().any(|range| {
                    let diff = range.end.clone() - range.start.clone();
                    diff.almost_zero()
                });
                // Should split if all these conditions hold
                if test1 && test2 && test3 {
                    let owned_items = std::mem::take(items);
                    let owned_ids = std::mem::take(ids);
                    self.split();
                    owned_items
                        .into_iter()
                        .zip(owned_ids)
                        .for_each(|(p, id)| self.insert(p, id));
                }
            }
            Self::Parent {
                ranges, children, ..
            } => {
                let mut idx: usize = 0;
                ranges.iter().enumerate().for_each(|(axis, range)| {
                    if T::two() * point[axis].clone()
                        >= range.start.clone() + range.end.clone()
                    {
                        idx ^= 1 << axis;
                    }
                });
                children[idx].insert(point, id);
            }
        };
    }
}

impl<T, Idx, const K: usize> Node<T, [(T, T); K], K, Idx>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    fn insert(&mut self, item: [(T, T); K], id: Idx) {
        match &mut *self {
            Self::Leaf {
                items,
                ids,
                capacity,
                depth,
                max_depth,
                ..
            } => {
                items.push(item);
                ids.push(id);
                let mut max_depth_reached = false;
                if let Some(max_depth) = max_depth {
                    if depth >= max_depth {
                        max_depth_reached = true;
                    }
                }
                if items.len() > capacity.get() && !max_depth_reached {
                    let owned_items = std::mem::take(items);
                    let owned_ids = std::mem::take(ids);
                    self.split();
                    owned_items
                        .into_iter()
                        .zip(owned_ids)
                        .for_each(|(p, id)| self.insert(p, id));
                }
            }
            Self::Parent { children, .. } => {
                for child in children.iter_mut() {
                    if child.ranges().covers(&item) {
                        child.insert(item.clone(), id);
                    }
                }
            }
        };
    }
}
impl<T, Item, Idx, const K: usize> KdTreeBase<T, Item, K, Idx>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    pub fn new(
        ranges: impl Into<[Range<T>; K]>,
        capacity: usize,
        max_depth: Option<usize>,
    ) -> Self {
        let capacity: NonZero<usize> =
            capacity.try_into().expect("Capacity must be nonzero");
        let root = Node::new(ranges.into(), capacity, 0, max_depth);
        Self {
            root,
            capacity,
            max_depth,
            count: 0,
        }
    }
}

impl<T, Idx, const K: usize> KdTreeBase<T, [T; K], K, Idx>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    pub fn insert(&mut self, item: [T; K], id: Idx) {
        if self
            .root
            .ranges()
            .iter()
            .enumerate()
            .all(|(axis, range)| range.contains(&item[axis]))
        {
            self.root.insert(item, id);
            self.count += 1;
        }
    }
}

impl<T, Idx, const K: usize> KdTreeBase<T, [(T, T); K], K, Idx>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
    Idx: Copy + Eq,
{
    pub fn insert(&mut self, item: [(T, T); K], id: Idx) {
        if self.root.ranges().covers(&item) {
            self.root.insert(item, id);
            self.count += 1;
        }
    }
}

pub type KdTree<T, const K: usize, Idx = usize> = KdTreeBase<T, [T; K], K, Idx>;

pub type Point2<T> = [T; 2];
pub type Point3<T> = [T; 3];
pub type Cell2<T> = [(T, T); 2];
pub type Cell3<T> = [(T, T); 3];

pub type Quadtree<T, Item = Point2<T>> = KdTreeBase<T, Item, 2>;
pub type QuadtreeNode<T, Item = Point2<T>> = Node<T, Item, 2>;
pub type Octree<T, Item = Point3<T>> = KdTreeBase<T, Item, 3>;
pub type OctreeNode<T, Item = Point3<T>> = Node<T, Item, 3>;

#[cfg(test)]
mod tests {}
