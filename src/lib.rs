#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::convert::From;
use std::num::NonZero;
use std::ops::{Add, Div, Range};

// TODO: Right now the tree data structure handles id:s internally based on
//       which order items are inserted. In many cases it might be desired to
//       have external control over which id corresponds to which point, as well
//       as support for other id types than usize.
// TODO: Implement a version that incrementally updates the tree when
//       points move

pub const MAX_DEPTH: usize = 32;

#[derive(Debug)]
pub struct KdTree<T, Item, const K: usize>
where
    [(); 1 << K]: Sized,
{
    pub root: Node<T, Item, K>,
    pub count: usize,
    pub max_depth: Option<usize>,
    #[allow(dead_code)]
    capacity: NonZero<usize>,
}

#[derive(Debug)]
pub enum Node<T, Item, const K: usize>
where
    [(); 1 << K]: Sized,
{
    Leaf {
        ranges: [Range<T>; K],
        items: Vec<Item>,
        ids: Vec<usize>,
        capacity: NonZero<usize>,
        depth: usize,
        max_depth: Option<usize>,
    },
    Parent {
        ranges: [Range<T>; K],
        children: Box<[Node<T, Item, K>; 1 << K]>,
        #[allow(dead_code)]
        capacity: NonZero<usize>,
        depth: usize,
        max_depth: Option<usize>,
    },
}

#[derive(Debug)]
pub enum ErrorKind {
    EmptyRegion,
    NodeAlreadySplit,
    OutOfBounds,
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

/*
impl<T, const K: usize> AsCoordinates<T, K> for [T; K]
where
    T: Copy,
{
    fn coordinates(&self) -> [T; K] {
        *self
    }
}
*/

pub trait Two<T> {
    fn two() -> T;
}

impl<T> Two<T> for T
where
    T: From<f32>,
{
    #[inline]
    fn two() -> T {
        2.0.into()
    }
}

pub trait Halveable:
    Sized + PartialOrd + Add<Output = Self> + Div<Output = Self> + Two<Self>
{
}

impl<T> Halveable for T where
    T: Sized + PartialOrd + Add<Output = Self> + Div<Output = Self> + Two<Self>
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

impl<T, Item, const K: usize> Node<T, Item, K>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
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
            items: vec![],
            ids: vec![],
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
    fn split(&mut self) -> Result<(), ErrorKind> {
        // If the range has 0 volume, it makes no sense to split.
        if self.ranges().iter().any(|range| range.is_empty()) {
            return Err(ErrorKind::EmptyRegion);
        }
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
                //new_self.insert_vec(items);
                //items.into_iter().for_each(|item| new_self.insert(item));
                unsafe {
                    std::ptr::write(self, new_self);
                };
                Ok(())
            }
            other => {
                unsafe {
                    std::ptr::write(self, other);
                };
                Err(ErrorKind::NodeAlreadySplit)
            }
        }
    }
}

impl<T, const K: usize> Node<T, [T; K], K>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
{
    fn insert(&mut self, point: [T; K], id: usize) {
        match &mut *self {
            Self::Leaf {
                items,
                ids,
                capacity,
                depth,
                max_depth,
                ..
            } => {
                items.push(point);
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
                    let _ = self.split();
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
                    if point[axis] >= range.mid_point() {
                        idx += 1 << axis;
                    }
                });
                children[idx].insert(point, id);
            }
        };
    }
}

impl<T, const K: usize> Node<T, [(T, T); K], K>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
{
    fn insert(&mut self, item: [(T, T); K], id: usize) {
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
                    let _ = self.split();
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
impl<T, Item, const K: usize> KdTree<T, Item, K>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
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

impl<T, const K: usize> KdTree<T, [T; K], K>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
{
    pub fn insert_point(&mut self, point: [T; K]) {
        if self
            .root
            .ranges()
            .iter()
            .enumerate()
            .all(|(axis, range)| range.contains(&point[axis]))
        {
            self.root.insert(point, self.count);
            self.count += 1;
        }
    }

    pub fn insert<Item>(&mut self, item: Item)
    where
        Item: Into<[T; K]>,
    {
        self.insert_point(item.into());
    }

    pub fn insert_slice<Item>(&mut self, items: &[Item])
    where
        for<'b> &'b Item: Into<[T; K]>,
    {
        for item in items {
            let point: [T; K] = item.into();
            self.insert_point(point);
        }
    }

    pub fn insert_vec(&mut self, vec: Vec<[T; K]>) {
        vec.into_iter().for_each(|p| self.insert_point(p));
    }
}

impl<T, const K: usize> KdTree<T, [(T, T); K], K>
where
    T: Halveable + Clone,
    [(); 1 << K]: Sized,
{
    pub fn insert_cell(&mut self, cell: [(T, T); K]) {
        if self.root.ranges().covers(&cell) {
            self.root.insert(cell, self.count);
            self.count += 1;
        }
    }
    pub fn insert<Item>(&mut self, item: Item)
    where
        Item: Into<[(T, T); K]>,
    {
        self.insert_cell(item.into());
    }

    pub fn insert_slice<Item>(&mut self, items: &[Item])
    where
        for<'b> &'b Item: Into<[(T, T); K]>,
    {
        for item in items {
            let cell: [(T, T); K] = item.into();
            self.insert_cell(cell);
        }
    }
}
pub type Point2<T> = [T; 2];
pub type Point3<T> = [T; 3];
pub type Cell2<T> = [(T, T); 2];
pub type Cell3<T> = [(T, T); 3];

pub type Quadtree<T, Item = Point2<T>> = KdTree<T, Item, 2>;
pub type QuadtreeNode<T, Item = Point2<T>> = Node<T, Item, 2>;
pub type Octtree<T, Item = Point3<T>> = KdTree<T, Item, 3>;
pub type OcttreeNode<T, Item = Point3<T>> = Node<T, Item, 3>;

#[cfg(test)]
mod tests {}
